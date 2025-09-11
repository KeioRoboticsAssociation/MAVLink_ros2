#include "stm32_mavlink_interface/robomaster_controller.hpp"
#include <chrono>
#include <cstring>

namespace stm32_mavlink_interface {

RobomasterController::RobomasterController(rclcpp::Node* node) : node_(node) {
    // Create ROS2 subscriptions
    motor_cmd_sub_ = node_->create_subscription<stm32_mavlink_interface::msg::RobomasterMotorCommand>(
        "robomaster/motor_command", 10,
        std::bind(&RobomasterController::motorCommandCallback, this, std::placeholders::_1));
    
    // Create ROS2 publishers
    motor_state_pub_ = node_->create_publisher<stm32_mavlink_interface::msg::RobomasterMotorState>(
        "robomaster/motor_state", 10);
    
    // Create ROS2 services
    set_config_srv_ = node_->create_service<stm32_mavlink_interface::srv::SetRobomasterMotorConfig>(
        "robomaster/set_motor_config",
        std::bind(&RobomasterController::setMotorConfigCallback, this, 
                  std::placeholders::_1, std::placeholders::_2));
    
    get_config_srv_ = node_->create_service<stm32_mavlink_interface::srv::GetRobomasterMotorConfig>(
        "robomaster/get_motor_config", 
        std::bind(&RobomasterController::getMotorConfigCallback, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(node_->get_logger(), "RoboMaster Controller initialized");
}

void RobomasterController::motorCommandCallback(const stm32_mavlink_interface::msg::RobomasterMotorCommand::SharedPtr msg) {
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
    
    RCLCPP_DEBUG(node_->get_logger(), "Received command for motor %d, mode %d", 
                 msg->motor_id, msg->control_mode);
}

void RobomasterController::setMotorConfigCallback(
    const std::shared_ptr<stm32_mavlink_interface::srv::SetRobomasterMotorConfig::Request> request,
    std::shared_ptr<stm32_mavlink_interface::srv::SetRobomasterMotorConfig::Response> response) {
    
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
    const std::shared_ptr<stm32_mavlink_interface::srv::GetRobomasterMotorConfig::Request> request,
    std::shared_ptr<stm32_mavlink_interface::srv::GetRobomasterMotorConfig::Response> response) {
    
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

void RobomasterController::handleMotorTelemetry(const mavlink_message_t& msg) {
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

void RobomasterController::handleMotorStatus(const mavlink_message_t& msg) {
    // Parse motor status message
    uint8_t motor_id = 1; // Would be extracted from message
    
    std::lock_guard<std::mutex> lock(motors_mutex_);
    ensureMotorExists(motor_id);
    MotorData& motor = motors_[motor_id];
    
    // Update status from message
    // motor.state.status = decoded_status;
    // motor.state.enabled = decoded_enabled;
    
    RCLCPP_DEBUG(node_->get_logger(), "Received status for motor %d", motor_id);
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
    
    RCLCPP_DEBUG(node_->get_logger(), "Received parameter: %s = %f", param_name.c_str(), param.param_value);
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
        auto time_since_command = now - motor.last_command_time;
        auto time_since_telemetry = now - motor.last_telemetry_time;
        
        if (time_since_command.seconds() > (COMMAND_TIMEOUT_MS / 1000.0)) {
            if (motor.state.enabled) {
                RCLCPP_WARN(node_->get_logger(), "Command timeout for motor %d", motor_id);
                motor.state.status = motor.state.STATUS_TIMEOUT;
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
        motor.last_command_time = node_->get_clock()->now();
        motor.last_telemetry_time = node_->get_clock()->now();
        motors_[motor_id] = motor;
    }
}

bool RobomasterController::isValidMotorId(uint8_t motor_id) const {
    return motor_id >= 1 && motor_id <= MAX_MOTORS;
}

void RobomasterController::buildMotorControlMessage(const stm32_mavlink_interface::msg::RobomasterMotorCommand& cmd,
                                                   mavlink_message_t& msg, uint8_t system_id, 
                                                   uint8_t component_id, uint8_t target_system) {
    // Build custom motor control message
    // This would use custom MAVLink message definitions
    
    // For now, use a standard message as placeholder
    mavlink_msg_manual_control_pack(system_id, component_id, &msg, target_system,
                                   (int16_t)(cmd.target_velocity_rps * 1000), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
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
                                             const stm32_mavlink_interface::msg::RobomasterMotorConfig& config) {
    // Queue parameter updates based on config
    // In practice, this would generate multiple parameter set messages
    
    (void)config; // Suppress unused parameter warning
    RCLCPP_INFO(node_->get_logger(), "Queuing parameter updates for motor %d", motor_id);
}

void RobomasterController::parametersToConfig(uint8_t motor_id, 
                                             stm32_mavlink_interface::msg::RobomasterMotorConfig& config) {
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

} // namespace stm32_mavlink_interface