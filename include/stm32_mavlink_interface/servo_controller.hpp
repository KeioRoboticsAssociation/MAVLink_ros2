#ifndef SERVO_CONTROLLER_HPP
#define SERVO_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <mutex>
#include "stm32_mavlink_interface/msg/servo_command.hpp"
#include "stm32_mavlink_interface/msg/servo_state.hpp"
#include "stm32_mavlink_interface/srv/set_servo_config.hpp"
#include "mavlink/c_library_v2/common/mavlink.h"

namespace stm32_mavlink_interface {

class ServoController {
public:
    ServoController(rclcpp::Node* node);
    ~ServoController() = default;
    
    // MAVLink message handlers
    void handleServoOutputRaw(const mavlink_servo_output_raw_t& servo_raw);
    void handleManualControl(const mavlink_manual_control_t& manual);
    void handleRCChannelsOverride(const mavlink_rc_channels_override_t& rc_override);
    
    // Get MAVLink messages to send
    bool getManualControlMessage(mavlink_message_t& msg, uint8_t system_id, uint8_t component_id, uint8_t target_system);
    bool getRCOverrideMessage(mavlink_message_t& msg, uint8_t system_id, uint8_t component_id, uint8_t target_system);
    
private:
    rclcpp::Node* node_;
    
    // ROS2 interfaces
    rclcpp::Subscription<stm32_mavlink_interface::msg::ServoCommand>::SharedPtr servo_cmd_sub_;
    rclcpp::Publisher<stm32_mavlink_interface::msg::ServoState>::SharedPtr servo_state_pub_;
    rclcpp::Service<stm32_mavlink_interface::srv::SetServoConfig>::SharedPtr servo_config_srv_;
    
    // Servo states
    struct ServoData {
        float target_angle_deg = 0.0f;
        float current_angle_deg = 0.0f;
        uint16_t pulse_us = 1500;
        bool enabled = false;
        uint8_t status = 0;
    };
    
    std::vector<ServoData> servo_states_;
    std::mutex servo_mutex_;
    
    // Callbacks
    void servoCommandCallback(const stm32_mavlink_interface::msg::ServoCommand::SharedPtr msg);
    void servoConfigCallback(const std::shared_ptr<stm32_mavlink_interface::srv::SetServoConfig::Request> request,
                            std::shared_ptr<stm32_mavlink_interface::srv::SetServoConfig::Response> response);
    
    void publishServoStates();
};

} // namespace stm32_mavlink_interface

#endif // SERVO_CONTROLLER_HPP
