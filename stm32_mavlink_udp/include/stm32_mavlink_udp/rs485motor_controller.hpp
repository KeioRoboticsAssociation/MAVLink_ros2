#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <queue>
#include <map>

#include "stm32_mavlink_msgs/msg/rs485_motor_command.hpp"
#include "stm32_mavlink_msgs/msg/rs485_motor_state.hpp"
#include "stm32_mavlink_msgs/msg/rs485_motor_config.hpp"

// MAVLink headers
#include "robomaster_motor/mavlink.h"

namespace stm32_mavlink_udp {

// Namespace aliases for shared messages
namespace msg = stm32_mavlink_msgs::msg;

class RS485MotorController {
public:
    explicit RS485MotorController(rclcpp::Node* node);
    ~RS485MotorController() = default;

    // ROS2 message handlers
    void motorCommandCallback(const msg::RS485MotorCommand::SharedPtr msg);

    // MAVLink message generation
    bool getMotorControlMessage(mavlink_message_t& msg, uint8_t system_id, uint8_t component_id, uint8_t target_system_id);

    // MAVLink message handlers
    void handleMotorStatus(const mavlink_message_t& msg);

    // Update function (called periodically)
    void update();

private:
    rclcpp::Node* node_;

    // ROS2 publishers and subscribers
    rclcpp::Publisher<msg::RS485MotorState>::SharedPtr motor_state_pub_;
    rclcpp::Subscription<msg::RS485MotorCommand>::SharedPtr motor_cmd_sub_;

    // Command queue for MAVLink transmission
    std::queue<msg::RS485MotorCommand> pending_commands_;

    // Current motor state
    msg::RS485MotorState current_state_;

    // State tracking (per motor ID)
    std::map<uint8_t, rclcpp::Time> last_motor_update_;   // Motor ID -> last update time
    std::map<uint8_t, bool> motor_connected_;             // Motor ID -> connection status
    std::map<uint8_t, msg::RS485MotorState> motor_states_; // Motor ID -> state

    // Constants
    static constexpr uint8_t RS485_MOTOR_ID_MIN = 30;  // RS485 motor ID range: 30-49
    static constexpr uint8_t RS485_MOTOR_ID_MAX = 49;
    static constexpr uint32_t MOTOR_TIMEOUT_MS = 2000;
};

} // namespace stm32_mavlink_udp
