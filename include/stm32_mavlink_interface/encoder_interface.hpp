#ifndef ENCODER_INTERFACE_HPP
#define ENCODER_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <mutex>
#include "stm32_mavlink_interface/msg/encoder_state.hpp"
#include "stm32_mavlink_interface/msg/encoder_config.hpp"
#include "stm32_mavlink_interface/srv/set_encoder_config.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "mavlink/c_library_v2/common/mavlink.h"

namespace stm32_mavlink_interface {

class EncoderInterface {
public:
    EncoderInterface(rclcpp::Node* node);
    ~EncoderInterface() = default;
    
    // MAVLink message handlers
    void handleAttitude(const mavlink_attitude_t& attitude);
    void handleParamValue(const mavlink_param_value_t& param_value);
    
    // Get MAVLink messages to send
    bool getEncoderConfigCommand(mavlink_message_t& msg, uint8_t system_id, uint8_t component_id, 
                                 uint8_t target_system, uint8_t encoder_id, uint8_t config_type, float value);
    
private:
    rclcpp::Node* node_;
    
    // ROS2 interfaces
    rclcpp::Publisher<stm32_mavlink_interface::msg::EncoderState>::SharedPtr encoder_state_pub_;
    rclcpp::Service<stm32_mavlink_interface::srv::SetEncoderConfig>::SharedPtr encoder_config_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_position_srv_;
    
    // Encoder states
    struct EncoderData {
        int32_t position = 0;
        float angle_rad = 0.0f;
        float angle_deg = 0.0f;
        uint32_t revolutions = 0;
        bool z_detected = false;
        uint8_t status = 0;
    };
    
    std::vector<EncoderData> encoder_states_;
    std::mutex encoder_mutex_;
    
    // Callbacks
    void encoderConfigCallback(const std::shared_ptr<stm32_mavlink_interface::srv::SetEncoderConfig::Request> request,
                               std::shared_ptr<stm32_mavlink_interface::srv::SetEncoderConfig::Response> response);
    void resetPositionCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    void publishEncoderStates();
};

} // namespace stm32_mavlink_interface

#endif // ENCODER_INTERFACE_HPP