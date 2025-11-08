#include "stm32_mavlink_uart/dcmotor_controller.hpp"
#include <regex>
#include <chrono>

namespace stm32_mavlink_uart {

DCMotorController::DCMotorController(rclcpp::Node* node)
    : node_(node) {

    // Create ROS2 publishers
    motor_state_pub_ = node_->create_publisher<msg::DCMotorState>("/dcmotor/state", 10);

    // Create ROS2 subscribers
    motor_cmd_sub_ = node_->create_subscription<msg::DCMotorCommand>(
        "/dcmotor/command", 10,
        std::bind(&DCMotorController::motorCommandCallback, this, std::placeholders::_1));

    // Create ROS2 services
    set_config_service_ = node_->create_service<srv::SetDCMotorConfig>(
        "/dcmotor/set_config",
        std::bind(&DCMotorController::setConfigCallback, this, std::placeholders::_1, std::placeholders::_2));

    get_config_service_ = node_->create_service<srv::GetDCMotorConfig>(
        "/dcmotor/get_config",
        std::bind(&DCMotorController::getConfigCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Initialize state for each DC motor (IDs 10-15 for direct PWM motors)
    for (uint8_t motor_id = DC_MOTOR_ID_MIN; motor_id <= DC_MOTOR_ID_MAX; motor_id++) {
        // Initialize motor state
        msg::DCMotorState state;
        state.motor_id = motor_id;
        state.position_rad = 0.0;
        state.velocity_rad_s = 0.0;
        state.current_a = 0.0;
        state.temperature_c = 0.0;
        state.status = 1; // STATUS_NOT_INITIALIZED
        state.enabled = false;
        motor_states_[motor_id] = state;

        // Initialize motor config with default values (matching STM32 setup)
        msg::DCMotorConfig config;
        config.motor_id = motor_id;
        config.mode = 0; // POSITION_CONTROL
        config.speed_kp = 0.8f;
        config.speed_ki = 0.0f;
        config.speed_kd = 0.0f;
        config.speed_max_integral = 10.0f;
        config.speed_max_output = 1.0f;
        config.position_kp = 3.0f;
        config.position_ki = 0.0f;
        config.position_kd = 0.0f;
        config.position_max_integral = 100.0f;
        config.position_max_output = 10.0f;
        config.max_speed_rad_s = 15.0f;
        config.max_acceleration_rad_s2 = 50.0f;
        config.use_position_limits = true;
        config.position_limit_min_rad = -314.159f; // -100 * pi
        config.position_limit_max_rad = 314.159f;  // +100 * pi
        config.watchdog_timeout_ms = 2000;
        config.control_period_ms = 10;
        motor_configs_[motor_id] = config;

        // Initialize connection tracking
        motor_connected_[motor_id] = false;
        last_motor_update_[motor_id] = node_->now();
    }

    // Initialize legacy state for backward compatibility
    current_state_ = motor_states_[DC_MOTOR_ID_MIN];
    current_config_ = motor_configs_[DC_MOTOR_ID_MIN];

    RCLCPP_INFO(node_->get_logger(), "DC Motor Controller initialized for motor IDs %d-%d",
                DC_MOTOR_ID_MIN, DC_MOTOR_ID_MAX);
}

void DCMotorController::motorCommandCallback(const msg::DCMotorCommand::SharedPtr msg) {
    // Validate motor ID is within DC motor range (10-15)
    if (msg->motor_id < DC_MOTOR_ID_MIN || msg->motor_id > DC_MOTOR_ID_MAX) {
        RCLCPP_WARN(node_->get_logger(), "Received command for motor ID %d, but DC motors use IDs %d-%d",
                   msg->motor_id, DC_MOTOR_ID_MIN, DC_MOTOR_ID_MAX);
        return;
    }

    if (msg->control_mode == 3) { // Duty-to-position control
        RCLCPP_INFO(node_->get_logger(), "DC Motor duty-to-position command: ID=%d, duty=%.3f, target_pos=%.3f, enabled=%s",
                   msg->motor_id, msg->target_value, msg->target_position_rad, msg->enabled ? "true" : "false");
    } else {
        RCLCPP_INFO(node_->get_logger(), "DC Motor command: ID=%d, mode=%d, value=%.3f, enabled=%s",
                   msg->motor_id, msg->control_mode, msg->target_value, msg->enabled ? "true" : "false");
    }

    // Add command to queue for MAVLink transmission
    pending_commands_.push(*msg);
}

bool DCMotorController::getMotorControlMessage(mavlink_message_t& msg, uint8_t system_id,
                                               uint8_t component_id, uint8_t target_system_id) {
    if (pending_commands_.empty()) {
        return false;
    }

    auto cmd = pending_commands_.front();
    pending_commands_.pop();

    // Use the new custom DC_MOTOR_CONTROL MAVLink message (ID 12002)
    mavlink_msg_dc_motor_control_pack(
        system_id,
        component_id,
        &msg,
        target_system_id,          // target_system
        1,                         // target_component
        cmd.motor_id,              // motor_id
        mapControlMode(cmd.control_mode),  // control_mode (0=PWM, 1=Speed, 2=Position, 3=Disabled)
        cmd.target_value,          // target_value (duty/speed/position depending on mode)
        0.0f,                      // speed_limit_rad_s (0 = use default)
        0.0f                       // acceleration_limit_rad_s2 (0 = use default)
    );

    RCLCPP_INFO(node_->get_logger(),
        "Sending DC_MOTOR_CONTROL: motor_id=%d, mode=%d, value=%.3f, enabled=%d",
        cmd.motor_id, cmd.control_mode, cmd.target_value, cmd.enabled);

    return true;
}

bool DCMotorController::getMotorConfigMessage(mavlink_message_t& msg, uint8_t system_id,
                                              uint8_t component_id, uint8_t target_system_id) {
    if (pending_configs_.empty()) {
        return false;
    }

    auto config = pending_configs_.front();
    pending_configs_.pop();

    // Send configuration as parameter set messages
    // This is a simplified approach - in practice, you might want to send multiple parameter messages
    mavlink_msg_param_set_pack(system_id, component_id, &msg,
                               target_system_id, 1,
                               "DC_SPEED_KP", config.speed_kp, MAV_PARAM_TYPE_REAL32);

    RCLCPP_INFO(node_->get_logger(), "Sending DC motor config parameter: speed_kp=%.3f", config.speed_kp);

    return true;
}

bool DCMotorController::getParameterRequestMessage(mavlink_message_t& msg, uint8_t system_id,
                                                   uint8_t component_id, uint8_t target_system_id) {
    // Request DC motor parameters periodically
    static int param_request_counter = 0;

    if (param_request_counter++ % 100 == 0) { // Every 10 seconds at 10Hz
        mavlink_msg_param_request_read_pack(system_id, component_id, &msg,
                                           target_system_id, 1,
                                           "DC_MOTOR_STATUS", -1);
        return true;
    }

    return false;
}

void DCMotorController::handleMotorStatusMAVLink(const mavlink_dc_motor_status_t& status) {
    uint8_t motor_id = status.motor_id;

    // Validate motor ID is within DC motor range (10-15)
    if (motor_id < DC_MOTOR_ID_MIN || motor_id > DC_MOTOR_ID_MAX) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
            "Received status for motor ID %d, but DC motors use IDs %d-%d",
            motor_id, DC_MOTOR_ID_MIN, DC_MOTOR_ID_MAX);
        return;
    }

    // Update motor state for the specific motor ID
    msg::DCMotorState state;
    state.motor_id = motor_id;
    state.position_rad = status.position_rad;
    state.velocity_rad_s = status.speed_rad_s;
    state.current_duty_cycle = status.duty_cycle;
    state.target_duty_cycle = status.duty_cycle; // Same as current for now
    state.target_velocity_rad_s = status.target_value; // Assume target value is velocity
    state.control_mode = status.control_mode;
    state.status = status.status;
    state.timestamp = status.timestamp_ms * 1000000ULL; // Convert ms to ns
    state.enabled = (status.control_mode != 3); // Mode 3 = DISABLED

    // Update stored state
    motor_states_[motor_id] = state;
    last_motor_update_[motor_id] = node_->now();
    motor_connected_[motor_id] = true;

    // Update legacy state for backward compatibility
    current_state_ = state;

    RCLCPP_DEBUG(node_->get_logger(),
        "DC Motor %d Status: pos=%.3f rad, vel=%.3f rad/s, duty=%.2f, mode=%d, status=%d",
        motor_id, status.position_rad, status.speed_rad_s, status.duty_cycle,
        status.control_mode, status.status);

    // Publish the state
    motor_state_pub_->publish(state);
}

void DCMotorController::handleMotorStatus(const mavlink_message_t& msg) {
    if (msg.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
        mavlink_statustext_t statustext;
        mavlink_msg_statustext_decode(&msg, &statustext);

        std::string status_str(statustext.text);

        // Look for DC motor status messages in the format:
        // "DC10: pos=1.23 vel=4.56 cur=0.78 temp=25 EN OK" or "DC11: ..."
        for (uint8_t motor_id = DC_MOTOR_ID_MIN; motor_id <= DC_MOTOR_ID_MAX; motor_id++) {
            if (status_str.find("DC" + std::to_string(motor_id) + ":") != std::string::npos) {
                parseMotorStatusText(status_str);
                last_motor_update_[motor_id] = node_->now();
                motor_connected_[motor_id] = true;
                break;
            }
        }
    }
}

void DCMotorController::handleMotorTelemetry(const mavlink_message_t& msg) {
    // Legacy telemetry handler - now using direct DC_MOTOR_STATUS messages
    RCLCPP_DEBUG(node_->get_logger(), "Received DC motor telemetry message ID: %d (legacy)", msg.msgid);
}

void DCMotorController::handleParameterValue(const mavlink_message_t& msg) {
    mavlink_param_value_t param_value;
    mavlink_msg_param_value_decode(&msg, &param_value);

    std::string param_name(param_value.param_id);

    // Update configuration based on received parameters
    if (param_name.find("DC_") == 0) {
        RCLCPP_INFO(node_->get_logger(), "Received DC motor parameter: %s = %.3f",
                   param_name.c_str(), param_value.param_value);

        // Update local config based on parameter name
        if (param_name == "DC_SPEED_KP") {
            current_config_.speed_kp = param_value.param_value;
        } else if (param_name == "DC_SPEED_KI") {
            current_config_.speed_ki = param_value.param_value;
        } else if (param_name == "DC_POSITION_KP") {
            current_config_.position_kp = param_value.param_value;
        }
        // Add more parameter mappings as needed
    }
}

void DCMotorController::handleCommandAck(const mavlink_message_t& msg) {
    mavlink_command_ack_t ack;
    mavlink_msg_command_ack_decode(&msg, &ack);

    if (ack.command == 31010) { // DC motor control command
        if (ack.result == MAV_RESULT_ACCEPTED) {
            RCLCPP_INFO(node_->get_logger(), "DC motor command accepted");
        } else {
            RCLCPP_WARN(node_->get_logger(), "DC motor command rejected: %d", ack.result);
        }
    }
}

void DCMotorController::update() {
    // Check for motor timeout for each motor
    auto now = node_->now();
    for (uint8_t motor_id = DC_MOTOR_ID_MIN; motor_id <= DC_MOTOR_ID_MAX; motor_id++) {
        if (motor_connected_[motor_id] &&
            (now - last_motor_update_[motor_id]).seconds() > (MOTOR_TIMEOUT_MS / 1000.0)) {
            motor_connected_[motor_id] = false;
            motor_states_[motor_id].status = 5; // STATUS_TIMEOUT
            RCLCPP_WARN(node_->get_logger(), "DC motor %d communication timeout", motor_id);
        }
    }

    // Update legacy state for backward compatibility
    current_state_ = motor_states_[DC_MOTOR_ID_MIN];
}

void DCMotorController::setConfigCallback(
    const std::shared_ptr<srv::SetDCMotorConfig::Request> request,
    std::shared_ptr<srv::SetDCMotorConfig::Response> response) {

    uint8_t motor_id = request->config.motor_id;
    if (motor_id < DC_MOTOR_ID_MIN || motor_id > DC_MOTOR_ID_MAX) {
        response->success = false;
        response->message = "Invalid motor ID: DC motors use IDs " +
                           std::to_string(DC_MOTOR_ID_MIN) + "-" + std::to_string(DC_MOTOR_ID_MAX);
        return;
    }

    // Update local config for this motor
    motor_configs_[motor_id] = request->config;
    current_config_ = request->config;  // Update legacy state

    // Add to pending configs for transmission
    pending_configs_.push(request->config);

    response->success = true;
    response->message = "DC motor configuration queued for transmission";

    RCLCPP_INFO(node_->get_logger(), "DC motor %d configuration updated", motor_id);
}

void DCMotorController::getConfigCallback(
    const std::shared_ptr<srv::GetDCMotorConfig::Request> request,
    std::shared_ptr<srv::GetDCMotorConfig::Response> response) {

    uint8_t motor_id = request->motor_id;
    if (motor_id < DC_MOTOR_ID_MIN || motor_id > DC_MOTOR_ID_MAX) {
        response->success = false;
        response->message = "Invalid motor ID: DC motors use IDs " +
                           std::to_string(DC_MOTOR_ID_MIN) + "-" + std::to_string(DC_MOTOR_ID_MAX);
        return;
    }

    response->success = true;
    response->config = motor_configs_[motor_id];
    response->message = "DC motor configuration retrieved";
}

void DCMotorController::parseMotorStatusText(const std::string& status_text) {
    // Parse status text like: "DC10: pos=1.23 vel=4.56 cur=0.78 temp=25 EN OK" or "DC11: ..."
    std::regex pattern(R"(DC(\d+):\s*pos=([-\d.]+)\s*vel=([-\d.]+)\s*cur=([-\d.]+)\s*temp=([-\d.]+)\s*EN\s*(OK|ERR))");
    std::smatch matches;

    if (std::regex_search(status_text, matches, pattern)) {
        uint8_t motor_id = std::stoi(matches[1].str());

        if (motor_id >= DC_MOTOR_ID_MIN && motor_id <= DC_MOTOR_ID_MAX) {
            motor_states_[motor_id].position_rad = std::stof(matches[2].str());
            motor_states_[motor_id].velocity_rad_s = std::stof(matches[3].str());
            motor_states_[motor_id].current_a = std::stof(matches[4].str());
            motor_states_[motor_id].temperature_c = std::stof(matches[5].str());
            motor_states_[motor_id].enabled = true;

            std::string status = matches[6].str();
            motor_states_[motor_id].status = (status == "OK") ? 0 : 2; // OK or ERROR

            // Update legacy state
            current_state_ = motor_states_[motor_id];

            RCLCPP_DEBUG(node_->get_logger(),
                        "DC motor state: pos=%.3f, vel=%.3f, cur=%.3f, temp=%.1f, status=%s",
                        current_state_.position_rad, current_state_.velocity_rad_s,
                        current_state_.current_a, current_state_.temperature_c, status.c_str());
        }
    } else {
        RCLCPP_WARN(node_->get_logger(), "Failed to parse DC motor status: %s", status_text.c_str());
    }
}

// Legacy telemetry handlers removed - now using direct DC_MOTOR_STATUS messages

uint8_t DCMotorController::mapControlMode(uint8_t ros_mode) {
    // Map ROS control modes to STM32 control modes
    // STM32 modes: 0=POSITION, 1=VELOCITY, 2=CURRENT/DUTY_CYCLE, 3=DISABLED
    // ROS2 modes match STM32 directly - no mapping needed
    switch (ros_mode) {
        case 0: return 0; // Position control
        case 1: return 1; // Velocity control
        case 2: return 2; // Current/duty control
        case 3: return 3; // Disabled
        default:
            RCLCPP_WARN(node_->get_logger(), "Unknown ROS control mode: %d, defaulting to Position", ros_mode);
            return 0; // Default to position control
    }
}

uint8_t DCMotorController::mapStatusFromText(const std::string& status_text) {
    // Map text status to MAVLink-aligned status codes
    if (status_text.find("OK") != std::string::npos) return 0; // STATUS_OK
    if (status_text.find("ERR") != std::string::npos) return 1; // STATUS_ERROR
    return 3; // STATUS_NOT_INITIALIZED
}

} // namespace stm32_mavlink_uart