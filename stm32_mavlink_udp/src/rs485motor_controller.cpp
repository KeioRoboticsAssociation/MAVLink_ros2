#include "stm32_mavlink_udp/rs485motor_controller.hpp"
#include <chrono>

namespace stm32_mavlink_udp {

RS485MotorController::RS485MotorController(rclcpp::Node* node)
    : node_(node) {

    // Create ROS2 publishers
    motor_state_pub_ = node_->create_publisher<msg::RS485MotorState>("/rs485motor/state", 10);

    // Create ROS2 subscribers
    motor_cmd_sub_ = node_->create_subscription<msg::RS485MotorCommand>(
        "/rs485motor/command", 10,
        std::bind(&RS485MotorController::motorCommandCallback, this, std::placeholders::_1));

    // Initialize state for each RS485 motor (IDs 30-49)
    for (uint8_t motor_id = RS485_MOTOR_ID_MIN; motor_id <= RS485_MOTOR_ID_MAX; motor_id++) {
        // Initialize motor state
        msg::RS485MotorState state;
        state.motor_id = motor_id;
        state.device_id = 0;
        state.motor_index = 0;
        state.control_mode = 0;
        state.status = 3; // NOT_INITIALIZED
        state.error_code = 0;
        state.current_position_rotations = 0.0;
        state.current_velocity_rps = 0.0;
        state.target_velocity_rps = 0.0;
        state.acceleration_rps2 = 0.0;
        state.timestamp_ms = 0;
        motor_states_[motor_id] = state;

        // Initialize connection tracking
        motor_connected_[motor_id] = false;
        last_motor_update_[motor_id] = node_->now();
    }

    // Initialize legacy state for backward compatibility
    current_state_ = motor_states_[RS485_MOTOR_ID_MIN];

    RCLCPP_INFO(node_->get_logger(), "RS485 Motor Controller initialized for motor IDs %d-%d",
                RS485_MOTOR_ID_MIN, RS485_MOTOR_ID_MAX);
}

void RS485MotorController::motorCommandCallback(const msg::RS485MotorCommand::SharedPtr msg) {
    // Validate motor ID is within RS485 motor range (30-49)
    if (msg->motor_id < RS485_MOTOR_ID_MIN || msg->motor_id > RS485_MOTOR_ID_MAX) {
        RCLCPP_WARN(node_->get_logger(), "Received command for motor ID %d, but RS485 motors use IDs %d-%d",
                   msg->motor_id, RS485_MOTOR_ID_MIN, RS485_MOTOR_ID_MAX);
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "RS485 Motor command: ID=%d, mode=%d, vel=%.3f, pos=%.3f, accel=%.3f, enabled=%s",
               msg->motor_id, msg->control_mode, msg->target_velocity, msg->target_position,
               msg->acceleration, msg->enabled ? "true" : "false");

    // Add command to queue for MAVLink transmission
    pending_commands_.push(*msg);
}

bool RS485MotorController::getMotorControlMessage(mavlink_message_t& msg, uint8_t system_id,
                                                   uint8_t component_id, uint8_t target_system_id) {
    if (pending_commands_.empty()) {
        return false;
    }

    auto cmd = pending_commands_.front();
    pending_commands_.pop();

    // Pack RS485 motor control message using MAVLink command_long
    // Using command ID 31020 for RS485 motor control
    float param1 = static_cast<float>(cmd.motor_id);
    float param2 = static_cast<float>(cmd.control_mode);
    float param3 = cmd.target_position;
    float param4 = cmd.target_velocity;
    float param5 = cmd.acceleration;
    float param6 = cmd.enabled ? 1.0f : 0.0f;
    float param7 = 0.0f; // Reserved

    mavlink_msg_command_long_pack(system_id, component_id, &msg,
                                  target_system_id, 1,
                                  31020, 0,
                                  param1, param2, param3, param4, param5, param6, param7);

    return true;
}

void RS485MotorController::handleMotorStatus(const mavlink_message_t& msg) {
    // Decode RS485 motor status message (MAVLink message ID 12005)
    mavlink_rs485_motor_status_t mavlink_status;
    mavlink_msg_rs485_motor_status_decode(&msg, &mavlink_status);

    // Validate motor ID
    uint8_t motor_id = mavlink_status.motor_id;
    if (motor_id < RS485_MOTOR_ID_MIN || motor_id > RS485_MOTOR_ID_MAX) {
        RCLCPP_WARN(node_->get_logger(), "Received RS485 motor status for invalid ID %d (valid range: %d-%d)",
                   motor_id, RS485_MOTOR_ID_MIN, RS485_MOTOR_ID_MAX);
        return;
    }

    // Convert MAVLink message to ROS2 message
    msg::RS485MotorState state;
    state.motor_id = mavlink_status.motor_id;
    state.device_id = mavlink_status.device_id;
    state.motor_index = mavlink_status.motor_index;
    state.control_mode = mavlink_status.control_mode;
    state.status = mavlink_status.status;
    state.error_code = mavlink_status.error_code;
    state.current_position_rotations = mavlink_status.current_position_rotations;
    state.current_velocity_rps = mavlink_status.current_velocity_rps;
    state.target_velocity_rps = mavlink_status.target_velocity_rps;
    state.acceleration_rps2 = mavlink_status.acceleration_rps2;
    state.timestamp_ms = mavlink_status.timestamp_ms;

    // Update motor state tracking
    motor_states_[motor_id] = state;
    last_motor_update_[motor_id] = node_->now();
    motor_connected_[motor_id] = true;

    // Publish state
    motor_state_pub_->publish(state);

    // Log detailed status (at debug level to avoid spam)
    RCLCPP_DEBUG(node_->get_logger(),
        "RS485 Motor %d: device_id=%d, motor_index=%d, pos=%.3f rot, vel=%.3f rps, status=%d, error=%d",
        motor_id, state.device_id, state.motor_index,
        state.current_position_rotations, state.current_velocity_rps,
        state.status, state.error_code);

    // Log warnings for error conditions
    if (state.status != 0) { // 0 = OK
        const char* status_str = "UNKNOWN";
        switch (state.status) {
            case 1: status_str = "ERROR"; break;
            case 2: status_str = "TIMEOUT"; break;
            case 3: status_str = "NOT_INITIALIZED"; break;
        }
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
            "RS485 Motor %d status: %s (error_code=%d)", motor_id, status_str, state.error_code);
    }
}

void RS485MotorController::update() {
    // Check for motor timeouts
    auto now = node_->now();
    for (uint8_t motor_id = RS485_MOTOR_ID_MIN; motor_id <= RS485_MOTOR_ID_MAX; motor_id++) {
        if (motor_connected_[motor_id]) {
            auto time_since_update = (now - last_motor_update_[motor_id]).seconds() * 1000.0;
            if (time_since_update > MOTOR_TIMEOUT_MS) {
                motor_connected_[motor_id] = false;
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                    "RS485 Motor %d connection timeout (%.0f ms since last update)",
                    motor_id, time_since_update);
            }
        }
    }
}

} // namespace stm32_mavlink_udp
