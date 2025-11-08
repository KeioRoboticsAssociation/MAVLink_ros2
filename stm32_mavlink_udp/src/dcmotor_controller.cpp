#include "stm32_mavlink_udp/dcmotor_controller.hpp"
#include <regex>
#include <chrono>

namespace stm32_mavlink_udp {

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

    // Pack DC motor control message (using custom MAVLink message ID)
    // Using MAVLink message ID 184 for DC motor control (extending the RoboMaster range)
    uint8_t payload[8];
    payload[0] = cmd.motor_id;
    payload[1] = mapControlMode(cmd.control_mode);

    // Pack target value as 4-byte float in little-endian format
    union { float f; uint8_t bytes[4]; } float_converter;
    float_converter.f = cmd.target_value;
    payload[2] = float_converter.bytes[0];
    payload[3] = float_converter.bytes[1];
    payload[4] = float_converter.bytes[2];
    payload[5] = float_converter.bytes[3];

    payload[6] = cmd.enabled ? 1 : 0;
    payload[7] = 0; // Reserved

    // Map control mode to appropriate STM32 command ID
    uint16_t command_id;
    switch (cmd.control_mode) {
        case 0: // Position control
            command_id = 31012;
            break;
        case 1: // Velocity control
            command_id = 31013;
            break;
        case 2: // Current/torque control
            command_id = 31014;
            break;
        case 3: // Duty-to-position control
            command_id = 31015;
            break;
        default:
            RCLCPP_WARN(node_->get_logger(), "Unknown control mode: %d, using position control", cmd.control_mode);
            command_id = 31012;
            break;
    }

    // Create MAVLink command with appropriate ID and parameters
    if (cmd.control_mode == 3) { // Duty-to-position control
        // For duty-to-position: param1=motor_id, param2=duty_cycle, param3=target_position_rad
        mavlink_msg_command_long_pack(system_id, component_id, &msg,
                                      target_system_id, 1,
                                      command_id, 0,
                                      cmd.motor_id, cmd.target_value, cmd.target_position_rad, cmd.enabled ? 1 : 0,
                                      0, 0, 0);
    } else {
        // Standard control modes
        mavlink_msg_command_long_pack(system_id, component_id, &msg,
                                      target_system_id, 1,
                                      command_id, 0,
                                      cmd.motor_id, cmd.target_value, cmd.enabled ? 1 : 0, 0,
                                      0, 0, 0);
    }

    if (cmd.control_mode == 3) { // Duty-to-position control
        RCLCPP_INFO(node_->get_logger(), "Sending DC motor duty-to-position command: ID=%d, duty=%.3f, target_pos=%.3f",
                   cmd.motor_id, cmd.target_value, cmd.target_position_rad);
    } else {
        RCLCPP_INFO(node_->get_logger(), "Sending DC motor control command: ID=%d, mode=%d, value=%.3f",
                   cmd.motor_id, cmd.control_mode, cmd.target_value);
    }

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
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_ATTITUDE:
            handleAttitudeMessage(msg);
            break;
        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
            handleServoOutputRawMessage(msg);
            break;
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
            handleLocalPositionMessage(msg);
            break;
        default:
            RCLCPP_DEBUG(node_->get_logger(), "Received unhandled DC motor telemetry message ID: %d", msg.msgid);
            break;
    }
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
                        "DC motor %d state: pos=%.3f, vel=%.3f, cur=%.3f, temp=%.1f, status=%s",
                        motor_id, current_state_.position_rad, current_state_.velocity_rad_s,
                        current_state_.current_a, current_state_.temperature_c, status.c_str());
        }
    } else {
        RCLCPP_WARN(node_->get_logger(), "Failed to parse DC motor status: %s", status_text.c_str());
    }
}

void DCMotorController::handleAttitudeMessage(const mavlink_message_t& msg) {
    mavlink_attitude_t attitude;
    mavlink_msg_attitude_decode(&msg, &attitude);

    // Extract position and velocity data as sent by STM32
    // roll = current position, pitch = target position, rollspeed = current velocity
    // Assume this is for motor ID 10 (default motor) if not specified in message
    uint8_t motor_id = DC_MOTOR_ID_MIN;
    motor_states_[motor_id].position_rad = attitude.roll;
    motor_states_[motor_id].velocity_rad_s = attitude.rollspeed;

    // Update connection status
    last_motor_update_[motor_id] = node_->now();
    motor_connected_[motor_id] = true;

    // Update legacy state
    current_state_ = motor_states_[motor_id];

    RCLCPP_DEBUG(node_->get_logger(),
                "DC Motor %d attitude: pos=%.3f rad, target=%.3f rad, vel=%.3f rad/s",
                motor_id, attitude.roll, attitude.pitch, attitude.rollspeed);
}

void DCMotorController::handleServoOutputRawMessage(const mavlink_message_t& msg) {
    mavlink_servo_output_raw_t servo_output;
    mavlink_msg_servo_output_raw_decode(&msg, &servo_output);

    // Extract motor status data as sent by STM32
    // port = motor_id, servo1_raw = current duty, servo2_raw = target duty
    // servo3_raw = enabled, servo4_raw = mode, servo5_raw = status
    uint8_t motor_id = servo_output.port;
    if (motor_id >= DC_MOTOR_ID_MIN && motor_id <= DC_MOTOR_ID_MAX) {
        // Convert duty cycle back from encoded format (500-2000 -> -1.0 to +1.0)
        motor_states_[motor_id].current_duty_cycle = ((servo_output.servo1_raw - 1250.0f) / 750.0f);
        motor_states_[motor_id].target_duty_cycle = ((servo_output.servo2_raw - 1250.0f) / 750.0f);

        motor_states_[motor_id].enabled = (servo_output.servo3_raw > 1250); // > 1.25 means enabled (new neutral)
        motor_states_[motor_id].control_mode = servo_output.servo4_raw / 500; // Decode mode
        motor_states_[motor_id].status = servo_output.servo5_raw / 200; // Decode status

        // Update connection status
        last_motor_update_[motor_id] = node_->now();
        motor_connected_[motor_id] = true;

        // Update legacy state
        current_state_ = motor_states_[motor_id];

        RCLCPP_DEBUG(node_->get_logger(),
                    "DC Motor %d servo output: duty=%.3f, target_duty=%.3f, enabled=%d, mode=%d, status=%d",
                    motor_id, motor_states_[motor_id].current_duty_cycle, motor_states_[motor_id].target_duty_cycle,
                    motor_states_[motor_id].enabled, motor_states_[motor_id].control_mode, motor_states_[motor_id].status);
    }
}

void DCMotorController::handleLocalPositionMessage(const mavlink_message_t& msg) {
    mavlink_local_position_ned_t local_pos;
    mavlink_msg_local_position_ned_decode(&msg, &local_pos);

    // Extract velocity data as sent by STM32
    // x = current speed, y = target speed
    // Assume this is for motor ID 10 (default motor) if not specified in message
    uint8_t motor_id = DC_MOTOR_ID_MIN;
    motor_states_[motor_id].velocity_rad_s = local_pos.x;
    motor_states_[motor_id].target_velocity_rad_s = local_pos.y;

    // Update connection status
    last_motor_update_[motor_id] = node_->now();
    motor_connected_[motor_id] = true;

    // Update legacy state
    current_state_ = motor_states_[motor_id];

    RCLCPP_DEBUG(node_->get_logger(),
                "DC Motor %d local position: vel=%.3f rad/s, target_vel=%.3f rad/s",
                motor_id,
                local_pos.x, local_pos.y);
}

uint8_t DCMotorController::mapControlMode(uint8_t ros_mode) {
    // Map ROS control modes to STM32 control modes
    switch (ros_mode) {
        case 0: return 2; // Position control
        case 1: return 1; // Velocity control
        case 2: return 0; // Current control
        case 3: return 4; // Duty-to-position control
        default: return 2; // Default to position control
    }
}

uint8_t DCMotorController::mapStatusFromText(const std::string& status_text) {
    if (status_text.find("OK") != std::string::npos) return 0;
    if (status_text.find("ERR") != std::string::npos) return 2;
    return 1; // NOT_INITIALIZED
}

} // namespace stm32_mavlink_udp