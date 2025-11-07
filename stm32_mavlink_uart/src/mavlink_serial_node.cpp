#include "stm32_mavlink_uart/mavlink_serial_node.hpp"
#include <chrono>
#include <cstring>

// Include RoboMaster message definitions for consistency
#include "stm32_mavlink_uart/robomaster_controller.hpp"

namespace stm32_mavlink_uart {

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
    dcmotor_controller_ = std::make_unique<DCMotorController>(this);

    // Create publishers
    diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics", 10);

    // CRITICAL FIX: Create RoboMaster subscription directly in main node for proper executor handling
    robomaster_cmd_sub_ = this->create_subscription<stm32_mavlink_msgs::msg::RobomasterMotorCommand>(
        "robomaster/motor_command", 10,
        [this](const stm32_mavlink_msgs::msg::RobomasterMotorCommand::SharedPtr msg) {
            RCLCPP_ERROR(this->get_logger(), "******************************************");
            RCLCPP_ERROR(this->get_logger(), "*** ROS2 CALLBACK TRIGGERED! Motor %d ***", msg->motor_id);
            RCLCPP_ERROR(this->get_logger(), "******************************************");
            robomaster_controller_->motorCommandCallback(msg);
        });

    RCLCPP_INFO(this->get_logger(), "Main node RoboMaster subscription created");
    
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
    int consecutive_empty_reads = 0;

    while (running_) {
        int bytes_read = read(serial_fd_, buffer, sizeof(buffer));

        if (bytes_read > 0) {
            consecutive_empty_reads = 0;  // Reset counter on successful read

            // Log raw data for debugging (first 50 bytes)
            static int debug_counter = 0;
            if (debug_counter++ % 100 == 0) {  // Every 100th read
                std::string hex_data;
                for (int j = 0; j < std::min(bytes_read, 50); j++) {
                    char hex_byte[4];
                    snprintf(hex_byte, sizeof(hex_byte), "%02X ", buffer[j]);
                    hex_data += hex_byte;
                }
                RCLCPP_INFO(this->get_logger(), "Raw data (%d bytes): %s", bytes_read, hex_data.c_str());
            }

            // Parse MAVLink messages (text-based parsing removed - now using proper MAVLink protocol)
            for (int i = 0; i < bytes_read; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &rx_msg_, &rx_status_)) {
                    handleMAVLinkMessage(rx_msg_);
                }
            }
        } else if (bytes_read < 0 && errno != EAGAIN) {
            RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", strerror(errno));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            consecutive_empty_reads++;
            // Log if we're not receiving data for extended periods
            if (consecutive_empty_reads % 1000 == 0) {  // Every 10 seconds
                RCLCPP_WARN(this->get_logger(), "No serial data received for %d read attempts (%.1f seconds)",
                           consecutive_empty_reads, consecutive_empty_reads * 10.0 / 1000.0);
            }
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
    // RCLCPP_INFO(this->get_logger(), "RX MAVLink message ID: %d, len: %d, sysid: %d", msg.msgid, msg.len, msg.sysid);

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
            
        // Custom motor control messages from new MAVLink protocol
        case MAVLINK_MSG_ID_DC_MOTOR_STATUS:
            handleDCMotorStatus(msg);
            break;

        case MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS:
            handleRobomasterMotorStatus(msg);
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
    static int telemetry_counter = 0;

    // Send multiple types of keepalive messages more frequently to keep STM32 active
    if (telemetry_counter % 10 == 0) { // Every second at 10Hz
        // Request data stream
        mavlink_msg_request_data_stream_pack(system_id_, component_id_, &msg,
                                           target_system_id_, target_component_id_,
                                           MAV_DATA_STREAM_ALL, 10, 1);
        sendMAVLinkMessage(msg);
        RCLCPP_INFO(this->get_logger(), "TX: Requesting data stream from STM32");
    }

    if (telemetry_counter % 20 == 5) { // Every 2 seconds, offset by 0.5 seconds
        // Send heartbeat to trigger STM32 response
        mavlink_msg_heartbeat_pack(system_id_, component_id_, &msg,
                                 MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_GENERIC,
                                 MAV_MODE_MANUAL_ARMED, 0, MAV_STATE_ACTIVE);
        sendMAVLinkMessage(msg);
        RCLCPP_INFO(this->get_logger(), "TX: Sending heartbeat to STM32");
    }

    if (telemetry_counter % 30 == 10) { // Every 3 seconds, offset
        // Request parameter list to trigger response
        mavlink_msg_param_request_list_pack(system_id_, component_id_, &msg,
                                           target_system_id_, target_component_id_);
        sendMAVLinkMessage(msg);
        RCLCPP_INFO(this->get_logger(), "TX: Requesting parameter list from STM32");
    }

    if (telemetry_counter % 40 == 15) { // Every 4 seconds, offset
        // Send command to request motor status
        mavlink_msg_command_long_pack(system_id_, component_id_, &msg,
                                    target_system_id_, target_component_id_,
                                    MAV_CMD_REQUEST_MESSAGE, 0,
                                    253, 0, 0, 0, 0, 0, 0); // Request STATUSTEXT messages
        sendMAVLinkMessage(msg);
        RCLCPP_INFO(this->get_logger(), "TX: Requesting status messages from STM32");
    }

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

    // Send DC motor commands if any
    if (dcmotor_controller_->getMotorControlMessage(msg, system_id_, component_id_, target_system_id_)) {
        RCLCPP_INFO(this->get_logger(), "Sending DC motor command, msgid: %d", msg.msgid);
        sendMAVLinkMessage(msg);
    }

    // Send DC motor configuration updates if any
    if (dcmotor_controller_->getMotorConfigMessage(msg, system_id_, component_id_, target_system_id_)) {
        sendMAVLinkMessage(msg);
    }

    // Send DC motor parameter requests if any
    if (dcmotor_controller_->getParameterRequestMessage(msg, system_id_, component_id_, target_system_id_)) {
        sendMAVLinkMessage(msg);
    }

    // Update controllers
    robomaster_controller_->update();
    dcmotor_controller_->update();

    publishDiagnostics();
    telemetry_counter++;
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

void MAVLinkSerialNode::handleDCMotorStatus(const mavlink_message_t& msg) {
    // Decode the DC motor status message
    mavlink_dc_motor_status_t dc_status;
    mavlink_msg_dc_motor_status_decode(&msg, &dc_status);

    RCLCPP_DEBUG(this->get_logger(),
        "DC Motor %d: pos=%.2f rad, vel=%.2f rad/s, duty=%.2f, mode=%d, status=%d",
        dc_status.motor_id, dc_status.position_rad, dc_status.speed_rad_s,
        dc_status.duty_cycle, dc_status.control_mode, dc_status.status);

    // Forward to DC motor controller
    dcmotor_controller_->handleMotorStatusMAVLink(dc_status);
}

void MAVLinkSerialNode::handleRobomasterMotorStatus(const mavlink_message_t& msg) {
    // Decode the RoboMaster motor status message
    mavlink_robomaster_motor_status_t rm_status;
    mavlink_msg_robomaster_motor_status_decode(&msg, &rm_status);

    RCLCPP_DEBUG(this->get_logger(),
        "RoboMaster Motor %d: pos=%.2f rad, vel=%.2f rad/s, mode=%d, status=%d",
        rm_status.motor_id, rm_status.current_position_rad, rm_status.current_speed_rad_s,
        rm_status.control_mode, rm_status.status);

    // Forward to RoboMaster controller
    robomaster_controller_->handleMotorStatusMAVLink(rm_status);
}

} // namespace stm32_mavlink_uart

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<stm32_mavlink_uart::MAVLinkSerialNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}