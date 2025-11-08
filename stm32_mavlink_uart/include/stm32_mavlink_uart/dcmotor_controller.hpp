#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <queue>
#include <map>

#include "stm32_mavlink_msgs/msg/dc_motor_command.hpp"
#include "stm32_mavlink_msgs/msg/dc_motor_state.hpp"
#include "stm32_mavlink_msgs/msg/dc_motor_config.hpp"
#include "stm32_mavlink_msgs/srv/set_dc_motor_config.hpp"
#include "stm32_mavlink_msgs/srv/get_dc_motor_config.hpp"

// MAVLink headers - Updated to use custom protocol
#include "robomaster_motor/mavlink.h"

namespace stm32_mavlink_uart {

// Namespace aliases for shared messages
namespace msg = stm32_mavlink_msgs::msg;
namespace srv = stm32_mavlink_msgs::srv;

class DCMotorController {
public:
    explicit DCMotorController(rclcpp::Node* node);
    ~DCMotorController() = default;

    // ROS2 message handlers
    void motorCommandCallback(const msg::DCMotorCommand::SharedPtr msg);

    // MAVLink message generation
    bool getMotorControlMessage(mavlink_message_t& msg, uint8_t system_id, uint8_t component_id, uint8_t target_system_id);
    bool getMotorConfigMessage(mavlink_message_t& msg, uint8_t system_id, uint8_t component_id, uint8_t target_system_id);
    bool getParameterRequestMessage(mavlink_message_t& msg, uint8_t system_id, uint8_t component_id, uint8_t target_system_id);

    // NEW: MAVLink custom protocol handler
    void handleMotorStatusMAVLink(const mavlink_dc_motor_status_t& status);

    // Legacy MAVLink message handlers (kept for compatibility)
    void handleMotorStatus(const mavlink_message_t& msg);
    void handleMotorTelemetry(const mavlink_message_t& msg);
    void handleParameterValue(const mavlink_message_t& msg);
    void handleCommandAck(const mavlink_message_t& msg);

    // Update function (called periodically)
    void update();

private:
    rclcpp::Node* node_;

    // ROS2 publishers and subscribers
    rclcpp::Publisher<msg::DCMotorState>::SharedPtr motor_state_pub_;
    rclcpp::Subscription<msg::DCMotorCommand>::SharedPtr motor_cmd_sub_;
    rclcpp::Service<srv::SetDCMotorConfig>::SharedPtr set_config_service_;
    rclcpp::Service<srv::GetDCMotorConfig>::SharedPtr get_config_service_;

    // Command queue for MAVLink transmission
    std::queue<msg::DCMotorCommand> pending_commands_;
    std::queue<msg::DCMotorConfig> pending_configs_;

    // Current motor state
    msg::DCMotorState current_state_;
    msg::DCMotorConfig current_config_;

    // Service callbacks
    void setConfigCallback(
        const std::shared_ptr<srv::SetDCMotorConfig::Request> request,
        std::shared_ptr<srv::SetDCMotorConfig::Response> response);

    void getConfigCallback(
        const std::shared_ptr<srv::GetDCMotorConfig::Request> request,
        std::shared_ptr<srv::GetDCMotorConfig::Response> response);

    // Helper functions
    void parseMotorStatusText(const std::string& status_text);
    uint8_t mapControlMode(uint8_t ros_mode);
    uint8_t mapStatusFromText(const std::string& status_text);

    // State tracking (per motor ID)
    std::map<uint8_t, rclcpp::Time> last_motor_update_;  // Motor ID -> last update time
    std::map<uint8_t, bool> motor_connected_;             // Motor ID -> connection status
    std::map<uint8_t, msg::DCMotorState> motor_states_;  // Motor ID -> state
    std::map<uint8_t, msg::DCMotorConfig> motor_configs_; // Motor ID -> config

    // Constants
    static constexpr uint8_t DC_MOTOR_ID_MIN = 10;  // DC motor ID range: 10-15 (direct PWM motors)
    static constexpr uint8_t DC_MOTOR_ID_MAX = 15;
    static constexpr uint32_t MOTOR_TIMEOUT_MS = 2000;
};

} // namespace stm32_mavlink_uart