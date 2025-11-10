#ifndef MAVLINK_UDP_NODE_HPP
#define MAVLINK_UDP_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include "stm32_mavlink_msgs/msg/robomaster_motor_command.hpp"
#include <thread>
#include <atomic>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

// MAVLink headers
#include "robomaster_motor/mavlink.h"

// Component interfaces (local)
#include "stm32_mavlink_udp/servo_controller.hpp"
#include "stm32_mavlink_udp/encoder_interface.hpp"
#include "stm32_mavlink_udp/robomaster_controller.hpp"
#include "stm32_mavlink_udp/dcmotor_controller.hpp"

namespace stm32_mavlink_udp {

// Namespace aliases for shared messages
namespace msg = stm32_mavlink_msgs::msg;
namespace srv = stm32_mavlink_msgs::srv;

class MAVLinkUDPNode : public rclcpp::Node {
public:
    MAVLinkUDPNode();
    ~MAVLinkUDPNode();

private:
    // UDP socket
    int udp_socket_;
    struct sockaddr_in local_addr_;
    struct sockaddr_in remote_addr_;
    std::string local_host_;
    int local_port_;
    std::string remote_host_;
    int remote_port_;
    bool is_server_mode_;

    // MAVLink
    uint8_t system_id_;
    uint8_t component_id_;
    uint8_t target_system_id_;
    uint8_t target_component_id_;
    mavlink_message_t rx_msg_;
    mavlink_status_t rx_status_;

    // Threading
    std::thread rx_thread_;
    std::thread tx_thread_;
    std::atomic<bool> running_;

    // Components (local UDP versions)
    std::unique_ptr<stm32_mavlink_udp::ServoController> servo_controller_;
    std::unique_ptr<stm32_mavlink_udp::EncoderInterface> encoder_interface_;
    std::unique_ptr<stm32_mavlink_udp::RobomasterController> robomaster_controller_;
    std::unique_ptr<stm32_mavlink_udp::DCMotorController> dcmotor_controller_;

    // Timers
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr telemetry_timer_;

    // Publishers
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diagnostics_pub_;

    // Subscriptions
    rclcpp::Subscription<stm32_mavlink_msgs::msg::RobomasterMotorCommand>::SharedPtr robomaster_cmd_sub_;

    // Methods
    bool openUDPSocket();
    void closeUDPSocket();
    bool configureUDPSocket();

    void rxThread();
    void txThread();

    void handleMAVLinkMessage(const mavlink_message_t& msg);
    void sendMAVLinkMessage(const mavlink_message_t& msg);

    void sendHeartbeat();
    void sendTelemetry();
    void publishDiagnostics();

    // MAVLink handlers (same as serial version)
    void handleHeartbeat(const mavlink_message_t& msg);
    void handleManualControl(const mavlink_message_t& msg);
    void handleRCChannelsOverride(const mavlink_message_t& msg);
    void handleCommandLong(const mavlink_message_t& msg);
    void handleParamValue(const mavlink_message_t& msg);
    void handleServoOutputRaw(const mavlink_message_t& msg);
    void handleAttitude(const mavlink_message_t& msg);
    void handleCommandAck(const mavlink_message_t& msg);

    // Custom motor control message handlers
    void handleDCMotorStatus(const mavlink_message_t& msg);
    void handleRobomasterMotorStatus(const mavlink_message_t& msg);
};

} // namespace stm32_mavlink_udp

#endif // MAVLINK_UDP_NODE_HPP