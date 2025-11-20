#include "stm32_mavlink_udp/mavlink_udp_node.hpp"
#include <chrono>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

namespace stm32_mavlink_udp {

MAVLinkUDPNode::MAVLinkUDPNode()
    : Node("mavlink_udp_node"),
      udp_socket_(-1),
      running_(false) {

    // Declare parameters
    this->declare_parameter("local_host", "0.0.0.0");
    this->declare_parameter("local_port", 14550);
    this->declare_parameter("remote_host", "192.168.11.4");
    this->declare_parameter("remote_port", 14550);
    this->declare_parameter("is_server_mode", true);
    this->declare_parameter("system_id", 255);
    this->declare_parameter("component_id", 1);
    this->declare_parameter("target_system_id", 1);
    this->declare_parameter("target_component_id", 1);

    // Get parameters
    local_host_ = this->get_parameter("local_host").as_string();
    local_port_ = this->get_parameter("local_port").as_int();
    remote_host_ = this->get_parameter("remote_host").as_string();
    remote_port_ = this->get_parameter("remote_port").as_int();
    is_server_mode_ = this->get_parameter("is_server_mode").as_bool();
    system_id_ = this->get_parameter("system_id").as_int();
    component_id_ = this->get_parameter("component_id").as_int();
    target_system_id_ = this->get_parameter("target_system_id").as_int();
    target_component_id_ = this->get_parameter("target_component_id").as_int();

    // Initialize components
    servo_controller_ = std::make_unique<stm32_mavlink_udp::ServoController>(this);
    encoder_interface_ = std::make_unique<stm32_mavlink_udp::EncoderInterface>(this);
    robomaster_controller_ = std::make_unique<stm32_mavlink_udp::RobomasterController>(this);
    dcmotor_controller_ = std::make_unique<stm32_mavlink_udp::DCMotorController>(this);
    rs485motor_controller_ = std::make_unique<stm32_mavlink_udp::RS485MotorController>(this);

    // Create publishers
    diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics", 10);

    // Create RoboMaster subscription
    robomaster_cmd_sub_ = this->create_subscription<stm32_mavlink_msgs::msg::RobomasterMotorCommand>(
        "robomaster/motor_command", 10,
        [this](const stm32_mavlink_msgs::msg::RobomasterMotorCommand::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "UDP RoboMaster command received: Motor %d", msg->motor_id);
            robomaster_controller_->motorCommandCallback(msg);
        });

    RCLCPP_INFO(this->get_logger(), "UDP node RoboMaster subscription created");

    // Open UDP socket
    if (!openUDPSocket()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open UDP socket: %s:%d", local_host_.c_str(), local_port_);
        throw std::runtime_error("Failed to open UDP socket");
    }

    // Start threads
    running_ = true;
    rx_thread_ = std::thread(&MAVLinkUDPNode::rxThread, this);
    tx_thread_ = std::thread(&MAVLinkUDPNode::txThread, this);

    // Create timers
    heartbeat_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&MAVLinkUDPNode::sendHeartbeat, this));

    telemetry_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MAVLinkUDPNode::sendTelemetry, this));

    RCLCPP_INFO(this->get_logger(), "MAVLink UDP Node initialized on %s:%d (server: %s)",
                local_host_.c_str(), local_port_, is_server_mode_ ? "true" : "false");
}

MAVLinkUDPNode::~MAVLinkUDPNode() {
    running_ = false;

    if (rx_thread_.joinable()) {
        rx_thread_.join();
    }
    if (tx_thread_.joinable()) {
        tx_thread_.join();
    }

    closeUDPSocket();
}

bool MAVLinkUDPNode::openUDPSocket() {
    udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket: %s", strerror(errno));
        return false;
    }

    return configureUDPSocket();
}

void MAVLinkUDPNode::closeUDPSocket() {
    if (udp_socket_ >= 0) {
        close(udp_socket_);
        udp_socket_ = -1;
    }
}

bool MAVLinkUDPNode::configureUDPSocket() {
    // Configure local address
    memset(&local_addr_, 0, sizeof(local_addr_));
    local_addr_.sin_family = AF_INET;
    local_addr_.sin_port = htons(local_port_);

    if (local_host_ == "0.0.0.0") {
        local_addr_.sin_addr.s_addr = INADDR_ANY;
    } else {
        if (inet_aton(local_host_.c_str(), &local_addr_.sin_addr) == 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid local IP address: %s", local_host_.c_str());
            return false;
        }
    }

    // Configure remote address
    memset(&remote_addr_, 0, sizeof(remote_addr_));
    remote_addr_.sin_family = AF_INET;
    remote_addr_.sin_port = htons(remote_port_);
    if (inet_aton(remote_host_.c_str(), &remote_addr_.sin_addr) == 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid remote IP address: %s", remote_host_.c_str());
        return false;
    }

    // Set socket options
    int reuse = 1;
    if (setsockopt(udp_socket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        RCLCPP_WARN(this->get_logger(), "Failed to set SO_REUSEADDR: %s", strerror(errno));
    }

    // Set non-blocking mode
    int flags = fcntl(udp_socket_, F_GETFL, 0);
    if (flags < 0 || fcntl(udp_socket_, F_SETFL, flags | O_NONBLOCK) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set non-blocking mode: %s", strerror(errno));
        return false;
    }

    // Bind to local address
    if (bind(udp_socket_, (struct sockaddr*)&local_addr_, sizeof(local_addr_)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to bind UDP socket to %s:%d: %s",
                     local_host_.c_str(), local_port_, strerror(errno));
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "UDP socket bound to %s:%d, remote: %s:%d",
                local_host_.c_str(), local_port_, remote_host_.c_str(), remote_port_);

    return true;
}

void MAVLinkUDPNode::rxThread() {
    uint8_t buffer[1024];
    struct sockaddr_in sender_addr;
    socklen_t sender_len = sizeof(sender_addr);
    int consecutive_empty_reads = 0;

    while (running_) {
        ssize_t bytes_received = recvfrom(udp_socket_, buffer, sizeof(buffer), 0,
                                         (struct sockaddr*)&sender_addr, &sender_len);

        if (bytes_received > 0) {
            consecutive_empty_reads = 0;

            // Log raw data for debugging (first 50 bytes)
            static int debug_counter = 0;
            if (debug_counter++ % 100 == 0) {
                std::string hex_data;
                for (int j = 0; j < std::min((int)bytes_received, 50); j++) {
                    char hex_byte[4];
                    snprintf(hex_byte, sizeof(hex_byte), "%02X ", buffer[j]);
                    hex_data += hex_byte;
                }
                RCLCPP_INFO(this->get_logger(), "UDP RX (%ld bytes): %s", bytes_received, hex_data.c_str());
            }

            // Update remote address from sender (for server mode)
            if (is_server_mode_) {
                remote_addr_ = sender_addr;
            }

            // Parse MAVLink packets and extract text motor status messages
            for (int i = 0; i < bytes_received; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &rx_msg_, &rx_status_)) {
                    handleMAVLinkMessage(rx_msg_);
                }
            }

        } else if (bytes_received < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            RCLCPP_ERROR(this->get_logger(), "UDP recvfrom error: %s", strerror(errno));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            consecutive_empty_reads++;
            if (consecutive_empty_reads % 1000 == 0) {
                RCLCPP_DEBUG(this->get_logger(), "No UDP data received for %d read attempts", consecutive_empty_reads);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

void MAVLinkUDPNode::txThread() {
    while (running_) {
        // TX operations are handled by timers and callbacks
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void MAVLinkUDPNode::handleMAVLinkMessage(const mavlink_message_t& msg) {
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

        // Custom motor control messages from MAVLink protocol
        case MAVLINK_MSG_ID_DC_MOTOR_STATUS:
            handleDCMotorStatus(msg);
            break;

        case MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS:
            handleRobomasterMotorStatus(msg);
            break;

        case MAVLINK_MSG_ID_RS485_MOTOR_STATUS:
            handleRS485MotorStatus(msg);
            break;

        default:
            break;
    }
}

void MAVLinkUDPNode::sendMAVLinkMessage(const mavlink_message_t& msg) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

    if (udp_socket_ >= 0) {
        ssize_t bytes_sent = sendto(udp_socket_, buffer, len, 0,
                                   (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
        if (bytes_sent < 0) {
            RCLCPP_ERROR(this->get_logger(), "UDP sendto error: %s", strerror(errno));
        }
    }
}

void MAVLinkUDPNode::sendHeartbeat() {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(system_id_, component_id_, &msg,
                              MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID,
                              MAV_MODE_MANUAL_ARMED, 0, MAV_STATE_ACTIVE);
    sendMAVLinkMessage(msg);
}

void MAVLinkUDPNode::sendTelemetry() {
    mavlink_message_t msg;

    // Send servo commands if any
    if (servo_controller_->getRCOverrideMessage(msg, system_id_, component_id_, target_system_id_)) {
        sendMAVLinkMessage(msg);
    }

    // Send RoboMaster motor commands if any
    if (robomaster_controller_->getMotorControlMessage(msg, system_id_, component_id_, target_system_id_)) {
        RCLCPP_INFO(this->get_logger(), "UDP TX: RoboMaster command, msgid: %d", msg.msgid);
        sendMAVLinkMessage(msg);
    }

    // Send DC motor commands if any
    if (dcmotor_controller_->getMotorControlMessage(msg, system_id_, component_id_, target_system_id_)) {
        RCLCPP_INFO(this->get_logger(), "UDP TX: DC motor command, msgid: %d", msg.msgid);
        sendMAVLinkMessage(msg);
    }

    // Send RS485 motor commands if any
    if (rs485motor_controller_->getMotorControlMessage(msg, system_id_, component_id_, target_system_id_)) {
        RCLCPP_INFO(this->get_logger(), "UDP TX: RS485 motor command, msgid: %d", msg.msgid);
        sendMAVLinkMessage(msg);
    }

    // Update controllers
    robomaster_controller_->update();
    dcmotor_controller_->update();
    rs485motor_controller_->update();

    publishDiagnostics();
}

void MAVLinkUDPNode::publishDiagnostics() {
    auto diag_msg = diagnostic_msgs::msg::DiagnosticStatus();
    diag_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diag_msg.name = "MAVLink UDP";
    diag_msg.message = "Operating normally";
    diag_msg.hardware_id = local_host_ + ":" + std::to_string(local_port_);

    diagnostics_pub_->publish(diag_msg);
}

// MAVLink handlers (delegate to controllers)
void MAVLinkUDPNode::handleHeartbeat(const mavlink_message_t& msg) {
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(&msg, &heartbeat);
    RCLCPP_DEBUG(this->get_logger(), "UDP Heartbeat from system %d", msg.sysid);
}

void MAVLinkUDPNode::handleManualControl(const mavlink_message_t& msg) {
    mavlink_manual_control_t manual;
    mavlink_msg_manual_control_decode(&msg, &manual);
    servo_controller_->handleManualControl(manual);
}

void MAVLinkUDPNode::handleRCChannelsOverride(const mavlink_message_t& msg) {
    mavlink_rc_channels_override_t rc_override;
    mavlink_msg_rc_channels_override_decode(&msg, &rc_override);
    servo_controller_->handleRCChannelsOverride(rc_override);
}

void MAVLinkUDPNode::handleServoOutputRaw(const mavlink_message_t& msg) {
    mavlink_servo_output_raw_t servo_raw;
    mavlink_msg_servo_output_raw_decode(&msg, &servo_raw);
    servo_controller_->handleServoOutputRaw(servo_raw);
}

void MAVLinkUDPNode::handleAttitude(const mavlink_message_t& msg) {
    mavlink_attitude_t attitude;
    mavlink_msg_attitude_decode(&msg, &attitude);
    encoder_interface_->handleAttitude(attitude);
}

void MAVLinkUDPNode::handleParamValue(const mavlink_message_t& msg) {
    mavlink_param_value_t param_value;
    mavlink_msg_param_value_decode(&msg, &param_value);
    encoder_interface_->handleParamValue(param_value);
    robomaster_controller_->handleParameterValue(msg);
}

void MAVLinkUDPNode::handleCommandLong(const mavlink_message_t& msg) {
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(&msg, &cmd);

    if (cmd.command >= 31000 && cmd.command <= 31002) {
        mavlink_message_t response_msg;
        encoder_interface_->getEncoderConfigCommand(response_msg, system_id_, component_id_,
                                                   target_system_id_, cmd.param1, cmd.param2, cmd.param3);
        sendMAVLinkMessage(response_msg);
    }
}

void MAVLinkUDPNode::handleCommandAck(const mavlink_message_t& msg) {
    mavlink_command_ack_t ack;
    mavlink_msg_command_ack_decode(&msg, &ack);
    robomaster_controller_->handleCommandAck(msg);
    RCLCPP_DEBUG(this->get_logger(), "UDP Command ACK: command=%d, result=%d", ack.command, ack.result);
}

void MAVLinkUDPNode::handleDCMotorStatus(const mavlink_message_t& msg) {
    // Decode the DC motor status message
    mavlink_dc_motor_status_t dc_status;
    mavlink_msg_dc_motor_status_decode(&msg, &dc_status);

    RCLCPP_DEBUG(this->get_logger(),
        "DC Motor %d: pos=%.2f rad, vel=%.2f rad/s, duty=%.2f, mode=%d, status=%d",
        dc_status.motor_id, dc_status.position_rad, dc_status.speed_rad_s,
        dc_status.duty_cycle, dc_status.control_mode, dc_status.status);

    // Forward to DC motor controller
    dcmotor_controller_->handleMotorStatus(msg);
}

void MAVLinkUDPNode::handleRobomasterMotorStatus(const mavlink_message_t& msg) {
    // Decode the RoboMaster motor status message
    mavlink_robomaster_motor_status_t rm_status;
    mavlink_msg_robomaster_motor_status_decode(&msg, &rm_status);

    RCLCPP_DEBUG(this->get_logger(),
        "RoboMaster Motor %d: pos=%.2f rad, vel=%.2f rad/s, mode=%d, status=%d",
        rm_status.motor_id, rm_status.current_position_rad, rm_status.current_speed_rad_s,
        rm_status.control_mode, rm_status.status);

    // Forward to RoboMaster controller
    robomaster_controller_->handleMotorStatus(msg);
}

void MAVLinkUDPNode::handleRS485MotorStatus(const mavlink_message_t& msg) {
    // Decode the RS485 motor status message
    mavlink_rs485_motor_status_t rs485_status;
    mavlink_msg_rs485_motor_status_decode(&msg, &rs485_status);

    RCLCPP_DEBUG(this->get_logger(),
        "RS485 Motor %d: device_id=%d, motor_index=%d, pos=%.2f rot, vel=%.2f rps, mode=%d, status=%d",
        rs485_status.motor_id, rs485_status.device_id, rs485_status.motor_index,
        rs485_status.current_position_rotations, rs485_status.current_velocity_rps,
        rs485_status.control_mode, rs485_status.status);

    // Forward to RS485 motor controller
    rs485motor_controller_->handleMotorStatus(msg);
}

} // namespace stm32_mavlink_udp

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<stm32_mavlink_udp::MAVLinkUDPNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}