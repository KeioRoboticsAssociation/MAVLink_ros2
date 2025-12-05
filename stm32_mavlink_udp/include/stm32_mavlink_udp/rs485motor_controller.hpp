#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <queue>
#include <map>

#include "stm32_mavlink_msgs/msg/rs485_motor_command.hpp"
#include "stm32_mavlink_msgs/msg/rs485_motor_state.hpp"
#include "stm32_mavlink_msgs/msg/rs485_motor_config.hpp"
#include "stm32_mavlink_msgs/msg/rs485_read_request.hpp"
#include "stm32_mavlink_msgs/msg/rs485_read_response.hpp"
#include "stm32_mavlink_msgs/msg/rs485_write_request.hpp"
#include "stm32_mavlink_msgs/msg/rs485_write_response.hpp"
#include "stm32_mavlink_msgs/msg/rs485_flash_save_request.hpp"
#include "stm32_mavlink_msgs/srv/read_rs485_param.hpp"
#include "stm32_mavlink_msgs/srv/write_rs485_param.hpp"
#include "stm32_mavlink_msgs/srv/flash_save_rs485.hpp"

// MAVLink headers
#include "robomaster_motor/mavlink.h"

namespace stm32_mavlink_udp {

// Namespace aliases for shared messages
namespace msg = stm32_mavlink_msgs::msg;
namespace srv = stm32_mavlink_msgs::srv;

class RS485MotorController {
public:
    explicit RS485MotorController(rclcpp::Node* node);
    ~RS485MotorController() = default;

    // ROS2 message handlers
    void motorCommandCallback(const msg::RS485MotorCommand::SharedPtr msg);
    void readRequestCallback(const msg::RS485ReadRequest::SharedPtr msg);
    void writeRequestCallback(const msg::RS485WriteRequest::SharedPtr msg);
    void flashSaveRequestCallback(const msg::RS485FlashSaveRequest::SharedPtr msg);

    // ROS2 service handlers
    void readParamService(const std::shared_ptr<srv::ReadRS485Param::Request> request,
                         std::shared_ptr<srv::ReadRS485Param::Response> response);
    void writeParamService(const std::shared_ptr<srv::WriteRS485Param::Request> request,
                          std::shared_ptr<srv::WriteRS485Param::Response> response);
    void flashSaveService(const std::shared_ptr<srv::FlashSaveRS485::Request> request,
                         std::shared_ptr<srv::FlashSaveRS485::Response> response);

    // MAVLink message generation
    bool getMotorControlMessage(mavlink_message_t& msg, uint8_t system_id, uint8_t component_id, uint8_t target_system_id);
    bool getReadRequestMessage(mavlink_message_t& msg, uint8_t system_id, uint8_t component_id, uint8_t target_system_id);
    bool getWriteRequestMessage(mavlink_message_t& msg, uint8_t system_id, uint8_t component_id, uint8_t target_system_id);
    bool getFlashSaveRequestMessage(mavlink_message_t& msg, uint8_t system_id, uint8_t component_id, uint8_t target_system_id);

    // MAVLink message handlers
    void handleMotorStatus(const mavlink_message_t& msg);
    void handleReadResponse(const mavlink_message_t& msg);
    void handleWriteResponse(const mavlink_message_t& msg);
    void handleCommandAck(const mavlink_message_t& msg);

    // Update function (called periodically)
    void update();

private:
    rclcpp::Node* node_;

    // ROS2 publishers and subscribers
    rclcpp::Publisher<msg::RS485MotorState>::SharedPtr motor_state_pub_;
    rclcpp::Publisher<msg::RS485ReadResponse>::SharedPtr read_response_pub_;
    rclcpp::Publisher<msg::RS485WriteResponse>::SharedPtr write_response_pub_;
    rclcpp::Subscription<msg::RS485MotorCommand>::SharedPtr motor_cmd_sub_;
    rclcpp::Subscription<msg::RS485ReadRequest>::SharedPtr read_request_sub_;
    rclcpp::Subscription<msg::RS485WriteRequest>::SharedPtr write_request_sub_;
    rclcpp::Subscription<msg::RS485FlashSaveRequest>::SharedPtr flash_save_request_sub_;

    // ROS2 services
    rclcpp::Service<srv::ReadRS485Param>::SharedPtr read_param_service_;
    rclcpp::Service<srv::WriteRS485Param>::SharedPtr write_param_service_;
    rclcpp::Service<srv::FlashSaveRS485>::SharedPtr flash_save_service_;

    // Command queue for MAVLink transmission
    std::queue<msg::RS485MotorCommand> pending_commands_;
    std::queue<msg::RS485ReadRequest> pending_read_requests_;
    std::queue<msg::RS485WriteRequest> pending_write_requests_;
    std::queue<msg::RS485FlashSaveRequest> pending_flash_save_requests_;

    // Pending read operations (for service responses)
    struct PendingRead {
        std::shared_ptr<srv::ReadRS485Param::Response> response;
        rclcpp::Time request_time;
    };
    std::map<uint32_t, PendingRead> pending_reads_; // key: (motor_id << 16) | address

    // Pending write operations (for service responses)
    struct PendingWrite {
        std::shared_ptr<srv::WriteRS485Param::Response> response;
        rclcpp::Time request_time;
    };
    std::map<uint32_t, PendingWrite> pending_writes_; // key: (motor_id << 16) | address

    // Pending flash save operations (for service responses)
    struct PendingFlashSave {
        std::shared_ptr<srv::FlashSaveRS485::Response> response;
        rclcpp::Time request_time;
    };
    std::map<uint8_t, PendingFlashSave> pending_flash_saves_; // key: motor_id

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
