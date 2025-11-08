#include "stm32_mavlink_uart/robomaster_controller.hpp"
#include <chrono>
#include <cstring>

namespace stm32_mavlink_uart {

RobomasterController::RobomasterController(rclcpp::Node* node) : node_(node) {
    // No subscription here - handled by main MAVLink node

    // Create ROS2 publishers
    motor_state_pub_ = node_->create_publisher<stm32_mavlink_msgs::msg::RobomasterMotorState>(
        "robomaster/motor_state", 10);
    
    // Create ROS2 services
    set_config_srv_ = node_->create_service<stm32_mavlink_msgs::srv::SetRobomasterMotorConfig>(
        "robomaster/set_motor_config",
        std::bind(&RobomasterController::setMotorConfigCallback, this, 
                  std::placeholders::_1, std::placeholders::_2));
    
    get_config_srv_ = node_->create_service<stm32_mavlink_msgs::srv::GetRobomasterMotorConfig>(
        "robomaster/get_motor_config", 
        std::bind(&RobomasterController::getMotorConfigCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(node_->get_logger(), "RoboMaster Controller initialized");
}

void RobomasterController::motorCommandCallback(const stm32_mavlink_msgs::msg::RobomasterMotorCommand::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(motors_mutex_);

    if (!isValidMotorId(msg->motor_id)) {
        RCLCPP_WARN(node_->get_logger(), "Invalid motor ID: %d", msg->motor_id);
        return;
    }
    
    ensureMotorExists(msg->motor_id);
    MotorData& motor = motors_[msg->motor_id];
    
    motor.last_command = *msg;
    motor.last_command_time = node_->get_clock()->now();
    motor.command_pending = true;
    
    RCLCPP_INFO(node_->get_logger(), "Received command for motor %d, mode %d, value=%.3f, enabled=%d",
                 msg->motor_id, msg->control_mode,
                 (msg->control_mode == 1 ? msg->target_velocity_rps :
                  msg->control_mode == 0 ? msg->target_current_ma : msg->target_position_rad),
                 msg->enabled);
}

void RobomasterController::setMotorConfigCallback(
    const std::shared_ptr<stm32_mavlink_msgs::srv::SetRobomasterMotorConfig::Request> request,
    std::shared_ptr<stm32_mavlink_msgs::srv::SetRobomasterMotorConfig::Response> response) {
    
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    if (!isValidMotorId(request->motor_id)) {
        response->success = false;
        response->message = "Invalid motor ID";
        return;
    }
    
    ensureMotorExists(request->motor_id);
    MotorData& motor = motors_[request->motor_id];
    
    motor.config = request->config;
    motor.config_pending = true;
    
    // Convert config to parameter updates (this would be sent via MAVLink)
    configToParameters(request->motor_id, request->config);
    
    response->success = true;
    response->message = "Configuration queued for transmission";
    
    RCLCPP_INFO(node_->get_logger(), "Configuration updated for motor %d", request->motor_id);
}

void RobomasterController::getMotorConfigCallback(
    const std::shared_ptr<stm32_mavlink_msgs::srv::GetRobomasterMotorConfig::Request> request,
    std::shared_ptr<stm32_mavlink_msgs::srv::GetRobomasterMotorConfig::Response> response) {
    
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    if (!isValidMotorId(request->motor_id)) {
        response->success = false;
        response->message = "Invalid motor ID";
        return;
    }
    
    auto it = motors_.find(request->motor_id);
    if (it == motors_.end()) {
        response->success = false;
        response->message = "Motor not found";
        return;
    }
    
    response->success = true;
    response->config = it->second.config;
    response->message = "Configuration retrieved successfully";
}

void RobomasterController::handleMotorTelemetry(const mavlink_message_t& /* msg */) {
    // Parse custom RoboMaster telemetry message from STM32
    // This would decode position, velocity, current, temperature data
    
    // For now, simulate parsing a standard MAVLink message
    // In practice, this would use custom message definitions
    
    uint8_t motor_id = 1; // Would be extracted from message
    
    std::lock_guard<std::mutex> lock(motors_mutex_);
    ensureMotorExists(motor_id);
    MotorData& motor = motors_[motor_id];
    
    motor.state.header.stamp = node_->get_clock()->now();
    motor.state.motor_id = motor_id;
    motor.last_telemetry_time = node_->get_clock()->now();
    motor.online = true;
    
    // Update state from message data
    // motor.state.current_position_rad = decoded_position;
    // motor.state.current_velocity_rps = decoded_velocity;
    // motor.state.current_milliamps = decoded_current;
    // motor.state.temperature_celsius = decoded_temperature;
    
    RCLCPP_DEBUG(node_->get_logger(), "Received telemetry for motor %d", motor_id);
}

void RobomasterController::handleMotorStatusMAVLink(const mavlink_robomaster_motor_status_t& motor_status) {
    // Handle the new custom MAVLink protocol message
    std::lock_guard<std::mutex> lock(motors_mutex_);
    ensureMotorExists(motor_status.motor_id);
    MotorData& motor = motors_[motor_status.motor_id];

    // Update motor state from MAVLink message (new custom protocol)
    motor.state.current_position_rad = motor_status.current_position_rad;
    motor.state.current_velocity_rps = motor_status.current_speed_rad_s;
    motor.state.control_mode = motor_status.control_mode;
    motor.state.status = motor_status.status;
    motor.state.current_milliamps = 0.0;  // Not available in new format
    motor.state.temperature_celsius = 0.0;  // Not available in new format
    motor.state.target_position_rad = 0.0;  // Not directly available
    motor.state.target_velocity_rps = 0.0;  // Not directly available
    motor.state.target_current_ma = 0.0;  // Not available in new format
    motor.state.enabled = (motor_status.control_mode != 3); // Mode 3 = DISABLED
    motor.state.error_count = 0;  // Not available in new format
    motor.state.timeout_count = 0;  // Not available in new format
    motor.state.overheat_count = 0;  // Not available in new format
    motor.state.header.stamp = node_->get_clock()->now();
    motor.state.motor_id = motor_status.motor_id;
    motor.last_update = node_->get_clock()->now();

    // Immediately publish the updated state for this specific motor
    motor_state_pub_->publish(motor.state);

    RCLCPP_DEBUG(node_->get_logger(),
        "RoboMaster Motor %d: pos=%.3f rad, vel=%.3f rad/s, mode=%d, status=%d",
        motor_status.motor_id, motor_status.current_position_rad,
        motor_status.current_speed_rad_s, motor_status.control_mode, motor_status.status);
}

void RobomasterController::handleMotorStatus(const mavlink_message_t& msg) {
    // Handle both STATUSTEXT (253) and proper ROBOMASTER_MOTOR_STATUS (12001) messages
    if (msg.msgid == MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS) {
        // Handle proper MAVLink motor status message
        mavlink_robomaster_motor_status_t motor_status;
        mavlink_msg_robomaster_motor_status_decode(&msg, &motor_status);

        std::lock_guard<std::mutex> lock(motors_mutex_);
        ensureMotorExists(motor_status.motor_id);
        MotorData& motor = motors_[motor_status.motor_id];

        // Update motor state from MAVLink message (new format)
        motor.state.current_position_rad = motor_status.current_position_rad;
        motor.state.current_velocity_rps = motor_status.current_speed_rad_s;
        // Note: current_duty_cycle and position_error_rad are not available in ROS message
        // These new fields from the updated MAVLink could be logged or used for diagnostics
        motor.state.control_mode = motor_status.control_mode;
        motor.state.status = motor_status.status;
        // Set some defaults for missing fields in new format
        motor.state.current_milliamps = 0.0;  // Not available in new format
        motor.state.temperature_celsius = 0.0;  // Not available in new format
        motor.state.target_position_rad = 0.0;  // Not directly available
        motor.state.target_velocity_rps = 0.0;  // Not directly available
        motor.state.target_current_ma = 0.0;  // Not available in new format
        motor.state.enabled = true;  // Assume enabled if receiving status
        motor.state.error_count = 0;  // Not available in new format
        motor.state.timeout_count = 0;  // Not available in new format
        motor.state.overheat_count = 0;  // Not available in new format
        motor.state.header.stamp = node_->get_clock()->now();
        motor.state.motor_id = motor_status.motor_id;
        motor.last_update = node_->get_clock()->now();

        // Immediately publish the updated state for this specific motor
        motor_state_pub_->publish(motor.state);
        return;
    }

    // Fallback: Parse text format from STATUSTEXT: "M5: pos= vel= cur=1851 temp=34 EN OK"
    // MAVLink STATUSTEXT message has severity(1) + text(49)
    if (msg.len < 10) return; // Need minimum message length

    // Extract text from STATUSTEXT message (skip severity byte)
    std::string text_msg(reinterpret_cast<const char*>(msg.payload64) + 1, msg.len - 1);

    // Parse the text format: "M5: pos= vel= cur=1851 temp=34 EN OK"
    if (text_msg.size() < 10 || text_msg[0] != 'M') return;

    // Extract motor ID
    // Parse motor ID from text (format changed for 20-29 range)
    // This legacy parsing may need adjustment based on actual message format
    uint8_t motor_id = text_msg[1] - '0';
    if (motor_id < ROBOMASTER_ID_MIN || motor_id > ROBOMASTER_ID_MAX) return; // Invalid motor ID

    std::lock_guard<std::mutex> lock(motors_mutex_);
    ensureMotorExists(motor_id);
    MotorData& motor = motors_[motor_id];

    // Parse the text for current and temperature values
    size_t cur_pos = text_msg.find("cur=");
    size_t temp_pos = text_msg.find("temp=");
    size_t en_pos = text_msg.find(" EN ");
    size_t ok_pos = text_msg.find(" OK");
    size_t err_pos = text_msg.find(" ERR");

    if (cur_pos != std::string::npos && temp_pos != std::string::npos &&
        en_pos != std::string::npos && (ok_pos != std::string::npos || err_pos != std::string::npos)) {

        // Extract current value
        std::string cur_str = text_msg.substr(cur_pos + 4);
        size_t cur_end = cur_str.find(' ');
        if (cur_end != std::string::npos) {
            int current = std::stoi(cur_str.substr(0, cur_end));
            motor.state.current_milliamps = current;
        }

        // Extract temperature value
        std::string temp_str = text_msg.substr(temp_pos + 5);
        size_t temp_end = temp_str.find(' ');
        if (temp_end != std::string::npos) {
            int temperature = std::stoi(temp_str.substr(0, temp_end));
            motor.state.temperature_celsius = temperature;
        }

        // Set motor status based on EN OK/ERR
        motor.state.enabled = true;
        if (ok_pos != std::string::npos) {
            motor.state.status = motor.state.STATUS_OK;
        } else if (err_pos != std::string::npos) {
            motor.state.status = motor.state.STATUS_CAN_ERROR; // General CAN error
        }

        // Position and velocity are not provided in text format, keep previous values
        // or set to defaults if first time
        if (motor.last_telemetry_time.seconds() == 0) {
            motor.state.current_position_rad = 0.0;
            motor.state.current_velocity_rps = 0.0;
        }

        // Update timestamps
        motor.state.header.stamp = node_->get_clock()->now();
        motor.state.motor_id = motor_id;
        motor.last_telemetry_time = node_->get_clock()->now();
        motor.online = true;

        RCLCPP_DEBUG(node_->get_logger(),
                    "Motor %d status: cur=%d, temp=%d, enabled=%d",
                    motor_id, motor.state.current_milliamps, motor.state.temperature_celsius, motor.state.enabled);

        // Immediately publish the updated state
        motor_state_pub_->publish(motor.state);
    } else {
        RCLCPP_WARN(node_->get_logger(), "Could not parse motor status text: %s", text_msg.c_str());
    }
}

void RobomasterController::handleParameterValue(const mavlink_message_t& msg) {
    mavlink_param_value_t param;
    mavlink_msg_param_value_decode(&msg, &param);
    
    // Parse parameter name to extract motor ID and parameter type
    std::string param_name(param.param_id);
    
    // Example: "MOTOR1_VEL_KP" -> motor_id=1, param="VEL_KP"
    if (param_name.size() >= 6 && param_name.substr(0, 5) == "MOTOR") {
        uint8_t motor_id = param_name[5] - '0';
        if (isValidMotorId(motor_id)) {
            updateMotorParameter(motor_id, param_name, param.param_value);
        }
    }
    
    RCLCPP_INFO(node_->get_logger(), "Received parameter: %s = %f", param_name.c_str(), param.param_value);
}

void RobomasterController::handleCommandAck(const mavlink_message_t& msg) {
    mavlink_command_ack_t ack;
    mavlink_msg_command_ack_decode(&msg, &ack);
    
    RCLCPP_DEBUG(node_->get_logger(), "Command ACK: command=%d, result=%d", 
                 ack.command, ack.result);
}

bool RobomasterController::getMotorControlMessage(mavlink_message_t& msg, uint8_t system_id, 
                                                 uint8_t component_id, uint8_t target_system) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    // Find a motor with pending commands
    for (auto& [motor_id, motor] : motors_) {
        if (motor.command_pending) {
            buildMotorControlMessage(motor.last_command, msg, system_id, component_id, target_system);
            motor.command_pending = false;
            return true;
        }
    }
    
    return false; // No pending commands
}

bool RobomasterController::getMotorConfigMessage(mavlink_message_t& msg, uint8_t system_id,
                                                uint8_t component_id, uint8_t target_system) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    // Find a motor with pending config changes
    for (auto& [motor_id, motor] : motors_) {
        if (motor.config_pending) {
            // Send first parameter change for this motor
            buildParameterSetMessage(motor_id, "VEL_KP", motor.config.velocity_kp, msg,
                                   system_id, component_id, target_system);
            // In practice, would cycle through all parameters
            motor.config_pending = false;
            return true;
        }
    }
    
    return false; // No pending config changes
}

bool RobomasterController::getParameterRequestMessage(mavlink_message_t& msg, uint8_t system_id,
                                                     uint8_t component_id, uint8_t target_system) {
    std::lock_guard<std::mutex> lock(pending_requests_mutex_);
    
    if (!pending_param_requests_.empty()) {
        const auto& request = pending_param_requests_.front();
        
        // Build parameter request message
        mavlink_msg_param_request_read_pack(system_id, component_id, &msg,
                                          target_system, 1, // target component
                                          request.param_name.c_str(), -1);
        
        pending_param_requests_.erase(pending_param_requests_.begin());
        return true;
    }
    
    return false;
}

void RobomasterController::update() {
    checkCommandTimeouts();
    publishMotorStates();
}

void RobomasterController::publishMotorStates() {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    for (const auto& [motor_id, motor] : motors_) {
        motor_state_pub_->publish(motor.state);
    }
}

void RobomasterController::checkCommandTimeouts() {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    
    auto now = node_->get_clock()->now();
    
    for (auto& [motor_id, motor] : motors_) {
        auto time_since_telemetry = now - motor.last_telemetry_time;

        // Only check for command timeout if a command was actually sent
        if (motor.last_command_time.nanoseconds() > 0) {
            auto time_since_command = now - motor.last_command_time;
            if (time_since_command.seconds() > (COMMAND_TIMEOUT_MS / 1000.0)) {
                if (motor.state.enabled) {
                    RCLCPP_WARN(node_->get_logger(), "Command timeout for motor %d", motor_id);
                    motor.state.status = motor.state.STATUS_TIMEOUT;
                }
            }
        }

        if (time_since_telemetry.seconds() > 2.0) { // 2 second telemetry timeout
            motor.online = false;
        }
    }
}

void RobomasterController::ensureMotorExists(uint8_t motor_id) {
    if (motors_.find(motor_id) == motors_.end()) {
        MotorData motor;
        motor.state.motor_id = motor_id;
        motor.state.header.frame_id = "robomaster_motor_" + std::to_string(motor_id);
        // Don't set last_command_time until an actual command is received
        motor.last_telemetry_time = node_->get_clock()->now();
        motor.command_pending = false;
        motors_[motor_id] = motor;
    }
}

bool RobomasterController::isValidMotorId(uint8_t motor_id) const {
    return motor_id >= ROBOMASTER_ID_MIN && motor_id <= ROBOMASTER_ID_MAX;
}

void RobomasterController::buildMotorControlMessage(const stm32_mavlink_msgs::msg::RobomasterMotorCommand& cmd,
                                                   mavlink_message_t& msg, uint8_t system_id,
                                                   uint8_t component_id, uint8_t target_system) {
    // Validate motor ID
    if (cmd.motor_id < ROBOMASTER_ID_MIN || cmd.motor_id > ROBOMASTER_ID_MAX) {
        RCLCPP_ERROR(node_->get_logger(), "Invalid motor ID: %d", cmd.motor_id);
        return;
    }

    // Validate control mode (0-4 are valid)
    if (cmd.control_mode > 4) {
        RCLCPP_ERROR(node_->get_logger(), "Invalid control mode: %d (valid range: 0-4)", cmd.control_mode);
        return;
    }

    // Prepare values for new message format
    float duty_cycle = 0.0f;
    float target_position_rad = 0.0f;
    float target_speed_rad_s = 0.0f;
    uint32_t timeout_ms = 1000;  // Default timeout

    // Map control modes and values to new format
    switch (cmd.control_mode) {
        case 0: // CONTROL_MODE_OPEN_LOOP (legacy CONTROL_MODE_CURRENT)
            // For open loop control, convert current to duty cycle approximation
            duty_cycle = static_cast<float>(cmd.target_current_ma) / 16000.0f;  // Normalize to -1.0 to 1.0
            break;
        case 1: // CONTROL_MODE_SPEED (legacy CONTROL_MODE_VELOCITY)
            target_speed_rad_s = cmd.target_velocity_rps;
            break;
        case 2: // CONTROL_MODE_POSITION
            target_position_rad = cmd.target_position_rad;
            break;
        case 3: // CONTROL_MODE_DISABLED
            // Motor disabled - all values remain at defaults (zeros)
            break;
        case 4: // CONTROL_MODE_DUTY_TO_POSITION
            // Use duty cycle from current field and position target
            duty_cycle = static_cast<float>(cmd.target_current_ma) / 16000.0f;  // Duty cycle percentage
            target_position_rad = cmd.target_position_rad;  // Position to stop at
            timeout_ms = 5000;  // Longer timeout for positioning
            break;
        default:
            RCLCPP_ERROR(node_->get_logger(), "Unknown control mode: %d", cmd.control_mode);
            return;
    }

    // Use proper MAVLink message building with new signature
    // Function signature: target_system, target_component, motor_id, control_mode, duty_cycle, target_position_rad, target_speed_rad_s, timeout_ms
    mavlink_msg_robomaster_motor_control_pack(
        system_id, component_id, &msg,
        target_system,          // target_system
        1,                      // target_component
        cmd.motor_id,           // motor_id
        cmd.control_mode,       // control_mode
        duty_cycle,             // duty_cycle
        target_position_rad,    // target_position_rad
        target_speed_rad_s,     // target_speed_rad_s
        timeout_ms              // timeout_ms
    );

    RCLCPP_INFO(node_->get_logger(), "Built motor control message: ID=%d, mode=%d, duty=%.3f, pos=%.3f, vel=%.3f",
                 cmd.motor_id, cmd.control_mode, duty_cycle, target_position_rad, target_speed_rad_s);
}

void RobomasterController::buildParameterSetMessage(uint8_t motor_id, const std::string& param_name, 
                                                   float value, mavlink_message_t& msg,
                                                   uint8_t system_id, uint8_t component_id, 
                                                   uint8_t target_system) {
    std::string full_param_name = getParameterName(motor_id, param_name);
    
    mavlink_msg_param_set_pack(system_id, component_id, &msg, target_system, 1,
                              full_param_name.c_str(), value, MAV_PARAM_TYPE_REAL32);
}

void RobomasterController::configToParameters(uint8_t motor_id, 
                                             const stm32_mavlink_msgs::msg::RobomasterMotorConfig& config) {
    // Queue parameter updates based on config
    // In practice, this would generate multiple parameter set messages
    
    (void)config; // Suppress unused parameter warning
    RCLCPP_INFO(node_->get_logger(), "Queuing parameter updates for motor %d", motor_id);
}

void RobomasterController::parametersToConfig(uint8_t motor_id, 
                                             stm32_mavlink_msgs::msg::RobomasterMotorConfig& config) {
    // Convert received parameters back to config structure
    (void)motor_id; // Suppress unused parameter warning
    (void)config;   // Suppress unused parameter warning
}

std::string RobomasterController::getParameterName(uint8_t motor_id, const std::string& base_name) const {
    return "MOTOR" + std::to_string(motor_id) + "_" + base_name;
}

void RobomasterController::requestMotorParameter(uint8_t motor_id, const std::string& param_name) {
    std::lock_guard<std::mutex> lock(pending_requests_mutex_);
    
    PendingRequest request;
    request.motor_id = motor_id;
    request.param_name = getParameterName(motor_id, param_name);
    request.request_time = node_->get_clock()->now();
    
    pending_param_requests_.push_back(request);
}

void RobomasterController::updateMotorParameter(uint8_t motor_id, const std::string& param_name, float value) {
    std::lock_guard<std::mutex> lock(motors_mutex_);
    ensureMotorExists(motor_id);
    
    // Update motor config based on parameter name
    // This would parse the parameter name and update the appropriate config field
    
    RCLCPP_DEBUG(node_->get_logger(), "Updated parameter %s for motor %d: %f", 
                 param_name.c_str(), motor_id, value);
}

} // namespace stm32_mavlink_uart