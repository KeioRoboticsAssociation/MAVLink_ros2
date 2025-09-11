#ifndef MAVLINK_SERIAL_NODE_HPP
#define MAVLINK_SERIAL_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <thread>
#include <atomic>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

// MAVLink headers
#include "mavlink/c_library_v2/common/mavlink.h"

// Component interfaces
#include "servo_controller.hpp"
#include "encoder_interface.hpp"
#include "robomaster_controller.hpp"

namespace stm32_mavlink_interface {

class MAVLinkSerialNode : public rclcpp::Node {
public:
    MAVLinkSerialNode();
    ~MAVLinkSerialNode();

private:
    // Serial port
    int serial_fd_;
    std::string serial_port_;
    int baudrate_;
    uint8_t system_id_;
    uint8_t component_id_;
    uint8_t target_system_id_;
    uint8_t target_component_id_;

    // MAVLink
    mavlink_message_t rx_msg_;
    mavlink_status_t rx_status_;
    
    // Threading
    std::thread rx_thread_;
    std::thread tx_thread_;
    std::atomic<bool> running_;
    
    // Components
    std::unique_ptr<ServoController> servo_controller_;
    std::unique_ptr<EncoderInterface> encoder_interface_;
    std::unique_ptr<RobomasterController> robomaster_controller_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr telemetry_timer_;
    
    // Publishers
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diagnostics_pub_;
    
    // Methods
    bool openSerialPort();
    void closeSerialPort();
    bool configureSerialPort();
    
    void rxThread();
    void txThread();
    
    void handleMAVLinkMessage(const mavlink_message_t& msg);
    void sendMAVLinkMessage(const mavlink_message_t& msg);
    
    void sendHeartbeat();
    void sendTelemetry();
    void publishDiagnostics();
    
    // MAVLink handlers
    void handleHeartbeat(const mavlink_message_t& msg);
    void handleManualControl(const mavlink_message_t& msg);
    void handleRCChannelsOverride(const mavlink_message_t& msg);
    void handleCommandLong(const mavlink_message_t& msg);
    void handleParamValue(const mavlink_message_t& msg);
    void handleServoOutputRaw(const mavlink_message_t& msg);
    void handleAttitude(const mavlink_message_t& msg);
    void handleCommandAck(const mavlink_message_t& msg);
    
    // RoboMaster MAVLink handlers
    void handleRobomasterTelemetry(const mavlink_message_t& msg);
    void handleRobomasterStatus(const mavlink_message_t& msg);
};

} // namespace stm32_mavlink_interface

#endif // MAVLINK_SERIAL_NODE_HPP