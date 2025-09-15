#include "stm32_mavlink_interface/mavlink_serial_node.hpp"
#include <chrono>
#include <cstring>

// Include RoboMaster message definitions for consistency
#include "stm32_mavlink_interface/robomaster_controller.hpp"

namespace stm32_mavlink_interface {

MAVLinkSerialNode::MAVLinkSerialNode() 
    : Node("mavlink_serial_node"),
      serial_fd_(-1),
      running_(false) {
    
    // Declare parameters
    this->declare_parameter("serial_port", "/dev/ttyUSB0");
    this->declare_parameter("baudrate", 115200);
    this->declare_parameter("system_id", 255);
    this->declare_parameter("component_id", 1);
    this->declare_parameter("target_system_id", 1);
    this->declare_parameter("target_component_id", 1);
    
    // Get parameters
    serial_port_ = this->get_parameter("serial_port").as_string();
    baudrate_ = this->get_parameter("baudrate").as_int();
    system_id_ = this->get_parameter("system_id").as_int();
    component_id_ = this->get_parameter("component_id").as_int();
    target_system_id_ = this->get_parameter("target_system_id").as_int();
    target_component_id_ = this->get_parameter("target_component_id").as_int();
    
    // Initialize components
    servo_controller_ = std::make_unique<ServoController>(this);
    encoder_interface_ = std::make_unique<EncoderInterface>(this);
    robomaster_controller_ = std::make_unique<RobomasterController>(this);
    
    // Create publishers
    diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics", 10);
    
    // Open serial port
    if (!openSerialPort()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
        throw std::runtime_error("Failed to open serial port");
    }
    
    // Start threads
    running_ = true;
    rx_thread_ = std::thread(&MAVLinkSerialNode::rxThread, this);
    tx_thread_ = std::thread(&MAVLinkSerialNode::txThread, this);
    
    // Create timers
    heartbeat_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&MAVLinkSerialNode::sendHeartbeat, this));
    
    telemetry_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MAVLinkSerialNode::sendTelemetry, this));
    
    RCLCPP_INFO(this->get_logger(), "MAVLink Serial Node initialized on %s @ %d baud", 
                serial_port_.c_str(), baudrate_);
}

MAVLinkSerialNode::~MAVLinkSerialNode() {
    running_ = false;
    
    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }
    if (tx_thread_.joinable()) {
        tx_thread_.join();
    }
    
    closeSerialPort();
}

bool MAVLinkSerialNode::openSerialPort() {
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
        return false;
    }
    
    return configureSerialPort();
}

void MAVLinkSerialNode::closeSerialPort() {
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
    }
}

bool MAVLinkSerialNode::configureSerialPort() {
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    if (tcgetattr(serial_fd_, &tty) != 0) {
        return false;
    }
    
    // Set baud rate
    speed_t baud;
    switch (baudrate_) {
        case 9600: baud = B9600; break;
        case 19200: baud = B19200; break;
        case 38400: baud = B38400; break;
        case 57600: baud = B57600; break;
        case 115200: baud = B115200; break;
        default: baud = B115200; break;
    }
    
    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);
    
    // 8N1
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;  // 100ms timeout
    
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    
    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        return false;
    }
    
    return true;
}

void MAVLinkSerialNode::rxThread() {
    uint8_t buffer[1024];
    
    while (running_) {
        int bytes_read = read(serial_fd_, buffer, sizeof(buffer));
        
        if (bytes_read > 0) {
            for (int i = 0; i < bytes_read; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &rx_msg_, &rx_status_)) {
                    handleMAVLinkMessage(rx_msg_);
                }
            }
        } else if (bytes_read < 0 && errno != EAGAIN) {
            RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", strerror(errno));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

void MAVLinkSerialNode::txThread() {
    while (running_) {
        // TX operations are handled by timers and callbacks
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void MAVLinkSerialNode::handleMAVLinkMessage(const mavlink_message_t& msg) {
    RCLCPP_INFO(this->get_logger(), "RX MAVLink message ID: %d, len: %d, sysid: %d", msg.msgid, msg.len, msg.sysid);

    switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            handleHeartbeat(msg);
            break;
            
        case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
            handleServoOutputRaw(msg);
            break;
            
        case MAVLINK_MSG_ID_ATTITUDE:
            handleAttitude(msg);
            break;
            
        case MAVLINK_MSG_ID_PARAM_VALUE:
            handleParamValue(msg);
            break;
            
        case MAVLINK_MSG_ID_COMMAND_ACK:
            handleCommandAck(msg);
            break;
            
        // RoboMaster custom messages - using proper MAVLink definitions
        case MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL:
            // This would be handled on TX side
            break;

        case MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS:
            handleRobomasterStatus(msg);
            break;

        case 253: // MAVLINK_MSG_ID_STATUSTEXT - STM32 is using this ID for RoboMaster motor status
            RCLCPP_DEBUG(this->get_logger(), "Received RoboMaster motor status message (ID 253)");
            handleRobomasterStatus(msg);
            break;

        case MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONFIG:
            // Configuration response
            break;

        case MAVLINK_MSG_ID_ROBOMASTER_TELEMETRY:
            handleRobomasterTelemetry(msg);
            break;
            
        default:
            // Unknown message
            break;
    }
}

void MAVLinkSerialNode::sendMAVLinkMessage(const mavlink_message_t& msg) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    
    if (serial_fd_ >= 0) {
        int bytes_written = write(serial_fd_, buffer, len);
        if (bytes_written < 0) {
            RCLCPP_ERROR(this->get_logger(), "Serial write error: %s", strerror(errno));
        }
    }
}

void MAVLinkSerialNode::sendHeartbeat() {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(system_id_, component_id_, &msg,
                              MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID,
                              MAV_MODE_MANUAL_ARMED, 0, MAV_STATE_ACTIVE);
    sendMAVLinkMessage(msg);
}

void MAVLinkSerialNode::sendTelemetry() {
    mavlink_message_t msg;
    
    // Send servo commands if any
    if (servo_controller_->getRCOverrideMessage(msg, system_id_, component_id_, target_system_id_)) {
        sendMAVLinkMessage(msg);
    }
    
    // Send RoboMaster motor commands if any
    if (robomaster_controller_->getMotorControlMessage(msg, system_id_, component_id_, target_system_id_)) {
        RCLCPP_INFO(this->get_logger(), "send, %d", msg.msgid);
        sendMAVLinkMessage(msg);
    }
    
    // Send RoboMaster motor configuration updates if any
    if (robomaster_controller_->getMotorConfigMessage(msg, system_id_, component_id_, target_system_id_)) {
        sendMAVLinkMessage(msg);
    }
    
    // Send RoboMaster parameter requests if any
    if (robomaster_controller_->getParameterRequestMessage(msg, system_id_, component_id_, target_system_id_)) {
        sendMAVLinkMessage(msg);
    }
    
    // Update RoboMaster controller
    robomaster_controller_->update();
    
    publishDiagnostics();
}

void MAVLinkSerialNode::publishDiagnostics() {
    auto diag_msg = diagnostic_msgs::msg::DiagnosticStatus();
    diag_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diag_msg.name = "MAVLink Serial";
    diag_msg.message = "Operating normally";
    diag_msg.hardware_id = serial_port_;
    
    diagnostics_pub_->publish(diag_msg);
}

void MAVLinkSerialNode::handleHeartbeat(const mavlink_message_t& msg) {
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(&msg, &heartbeat);
    
    // Log heartbeat received
    RCLCPP_DEBUG(this->get_logger(), "Heartbeat from system %d", msg.sysid);
}

void MAVLinkSerialNode::handleManualControl(const mavlink_message_t& msg) {
    mavlink_manual_control_t manual;
    mavlink_msg_manual_control_decode(&msg, &manual);
    servo_controller_->handleManualControl(manual);
}

void MAVLinkSerialNode::handleRCChannelsOverride(const mavlink_message_t& msg) {
    mavlink_rc_channels_override_t rc_override;
    mavlink_msg_rc_channels_override_decode(&msg, &rc_override);
    servo_controller_->handleRCChannelsOverride(rc_override);
}

void MAVLinkSerialNode::handleServoOutputRaw(const mavlink_message_t& msg) {
    mavlink_servo_output_raw_t servo_raw;
    mavlink_msg_servo_output_raw_decode(&msg, &servo_raw);
    servo_controller_->handleServoOutputRaw(servo_raw);
}

void MAVLinkSerialNode::handleAttitude(const mavlink_message_t& msg) {
    mavlink_attitude_t attitude;
    mavlink_msg_attitude_decode(&msg, &attitude);
    encoder_interface_->handleAttitude(attitude);
}

void MAVLinkSerialNode::handleParamValue(const mavlink_message_t& msg) {
    mavlink_param_value_t param_value;
    mavlink_msg_param_value_decode(&msg, &param_value);
    encoder_interface_->handleParamValue(param_value);
    
    // Also forward to RoboMaster controller for motor parameters
    robomaster_controller_->handleParameterValue(msg);
}

void MAVLinkSerialNode::handleCommandLong(const mavlink_message_t& msg) {
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(&msg, &cmd);
    
    // Handle encoder configuration commands
    if (cmd.command >= 31000 && cmd.command <= 31002) {
        mavlink_message_t response_msg;
        encoder_interface_->getEncoderConfigCommand(response_msg, system_id_, component_id_, 
                                                   target_system_id_, cmd.param1, cmd.param2, cmd.param3);
        sendMAVLinkMessage(response_msg);
    }
}

void MAVLinkSerialNode::handleCommandAck(const mavlink_message_t& msg) {
    mavlink_command_ack_t ack;
    mavlink_msg_command_ack_decode(&msg, &ack);
    
    // Forward to RoboMaster controller
    robomaster_controller_->handleCommandAck(msg);
    
    RCLCPP_DEBUG(this->get_logger(), "Command ACK: command=%d, result=%d", ack.command, ack.result);
}

void MAVLinkSerialNode::handleRobomasterTelemetry(const mavlink_message_t& msg) {
    // Forward RoboMaster telemetry to controller
    robomaster_controller_->handleMotorTelemetry(msg);
}

void MAVLinkSerialNode::handleRobomasterStatus(const mavlink_message_t& msg) {
    // Forward RoboMaster status to controller
    robomaster_controller_->handleMotorStatus(msg);
}

} // namespace stm32_mavlink_interface

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<stm32_mavlink_interface::MAVLinkSerialNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}