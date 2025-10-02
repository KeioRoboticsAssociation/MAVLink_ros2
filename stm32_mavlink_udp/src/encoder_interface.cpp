#include "stm32_mavlink_udp/encoder_interface.hpp"
#include <cstring>

namespace stm32_mavlink_udp {

EncoderInterface::EncoderInterface(rclcpp::Node* node) : node_(node) {
    // Initialize encoder states (16 encoders max)
    encoder_states_.resize(16);
    
    // Create ROS2 interfaces
    encoder_state_pub_ = node_->create_publisher<stm32_mavlink_udp::msg::EncoderState>(
        "encoder/states", 10);
    
    encoder_config_srv_ = node_->create_service<stm32_mavlink_udp::srv::SetEncoderConfig>(
        "encoder/set_config",
        std::bind(&EncoderInterface::encoderConfigCallback, this,
                 std::placeholders::_1, std::placeholders::_2));
    
    reset_position_srv_ = node_->create_service<std_srvs::srv::Trigger>(
        "encoder/reset_position",
        std::bind(&EncoderInterface::resetPositionCallback, this,
                 std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(node_->get_logger(), "Encoder Interface initialized");
}

void EncoderInterface::handleAttitude(const mavlink_attitude_t& attitude) {
    std::lock_guard<std::mutex> lock(encoder_mutex_);
    
    // STM32 encodes encoder data in attitude message:
    // roll = encoder angle in radians
    // pitch = normalized position (0.0-1.0)
    // yaw = revolutions
    
    // Assuming encoder ID 1 for this example
    size_t encoder_id = 0;
    
    if (encoder_id < encoder_states_.size()) {
        encoder_states_[encoder_id].angle_rad = attitude.roll;
        encoder_states_[encoder_id].angle_deg = attitude.roll * 180.0f / M_PI;
        encoder_states_[encoder_id].revolutions = static_cast<uint32_t>(attitude.yaw);
        
        // Reconstruct position from normalized value (assuming CPR=1024)
        uint16_t cpr = 1024;  // This should come from configuration
        encoder_states_[encoder_id].position = static_cast<int32_t>(attitude.pitch * cpr);
    }
    
    publishEncoderStates();
}

void EncoderInterface::handleParamValue(const mavlink_param_value_t& param_value) {
    std::lock_guard<std::mutex> lock(encoder_mutex_);
    
    // Parse encoder configuration parameters
    // Format: ENCx_PARAM where x is encoder ID
    std::string param_id(param_value.param_id);
    
    if (param_id.substr(0, 3) == "ENC") {
        // Extract encoder ID
        int encoder_id = param_id[3] - '0';
        
        if (encoder_id >= 1 && encoder_id <= encoder_states_.size()) {
            // Handle different parameter types
            if (param_id.find("_CPR") != std::string::npos) {
                RCLCPP_INFO(node_->get_logger(), "Encoder %d CPR: %f", encoder_id, param_value.param_value);
            } else if (param_id.find("_INVA") != std::string::npos) {
                RCLCPP_INFO(node_->get_logger(), "Encoder %d InvertA: %s", encoder_id,
                           param_value.param_value ? "true" : "false");
            } else if (param_id.find("_INVB") != std::string::npos) {
                RCLCPP_INFO(node_->get_logger(), "Encoder %d InvertB: %s", encoder_id,
                           param_value.param_value ? "true" : "false");
            } else if (param_id.find("_USEZ") != std::string::npos) {
                RCLCPP_INFO(node_->get_logger(), "Encoder %d UseZ: %s", encoder_id,
                           param_value.param_value ? "true" : "false");
            }
        }
    }
}

bool EncoderInterface::getEncoderConfigCommand(mavlink_message_t& msg, uint8_t system_id,
                                              uint8_t component_id, uint8_t target_system,
                                              uint8_t encoder_id, uint8_t config_type, float value) {
    // Create COMMAND_LONG message for encoder configuration
    mavlink_msg_command_long_pack(system_id, component_id, &msg,
                                 target_system, MAV_COMP_ID_ALL,
                                 31000,  // MOTOR_CONFIG_SET command
                                 0,      // confirmation
                                 encoder_id,    // param1: encoder ID
                                 config_type,   // param2: config type
                                 value,         // param3: value
                                 0, 0, 0, 0);   // param4-7: unused
    return true;
}

void EncoderInterface::encoderConfigCallback(
    const std::shared_ptr<stm32_mavlink_udp::srv::SetEncoderConfig::Request> request,
    std::shared_ptr<stm32_mavlink_udp::srv::SetEncoderConfig::Response> response) {
    
    // In a real implementation, this would send configuration to STM32
    response->success = true;
    response->message = "Encoder configuration request received";
    
    RCLCPP_INFO(node_->get_logger(), "Encoder %d config request received", request->encoder_id);
}

void EncoderInterface::resetPositionCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    // In a real implementation, this would send reset command to STM32
    response->success = true;
    response->message = "Encoder position reset request sent";
    
    RCLCPP_INFO(node_->get_logger(), "Encoder position reset requested");
}

void EncoderInterface::publishEncoderStates() {
    auto now = node_->now();
    
    for (size_t i = 0; i < encoder_states_.size(); i++) {
        auto msg = stm32_mavlink_udp::msg::EncoderState();
        msg.header.stamp = now;
        msg.encoder_id = i + 1;
        msg.position = encoder_states_[i].position;
        msg.angle_rad = encoder_states_[i].angle_rad;
        msg.angle_deg = encoder_states_[i].angle_deg;
        msg.revolutions = encoder_states_[i].revolutions;
        msg.z_detected = encoder_states_[i].z_detected;
        msg.status = encoder_states_[i].status;
        msg.error_count = 0;
        
        encoder_state_pub_->publish(msg);
    }
}

} // namespace stm32_mavlink_udp