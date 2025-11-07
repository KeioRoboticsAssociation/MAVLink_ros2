#include "stm32_mavlink_uart/servo_controller.hpp"

namespace stm32_mavlink_uart {

ServoController::ServoController(rclcpp::Node* node) : node_(node) {
    // Initialize servo states (16 servos max)
    servo_states_.resize(16);
    
    // Create ROS2 interfaces
    servo_cmd_sub_ = node_->create_subscription<stm32_mavlink_msgs::msg::ServoCommand>(
        "servo/command", 10,
        std::bind(&ServoController::servoCommandCallback, this, std::placeholders::_1));
    
    servo_state_pub_ = node_->create_publisher<stm32_mavlink_msgs::msg::ServoState>(
        "servo/states", 10);
    
    servo_config_srv_ = node_->create_service<stm32_mavlink_msgs::srv::SetServoConfig>(
        "servo/set_config",
        std::bind(&ServoController::servoConfigCallback, this, 
                 std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(node_->get_logger(), "Servo Controller initialized");
}

void ServoController::handleServoOutputRaw(const mavlink_servo_output_raw_t& servo_raw) {
    std::lock_guard<std::mutex> lock(servo_mutex_);
    
    // 受信したことを必ずログ出力
    // RCLCPP_INFO(node_->get_logger(), 
    //     "SERVO_OUTPUT_RAW received: time=%u, port=%d, servo1=%d, servo2=%d, servo3=%d",
    //     static_cast<uint32_t>(servo_raw.time_usec), servo_raw.port, 
    //     servo_raw.servo1_raw, servo_raw.servo2_raw, servo_raw.servo3_raw);
    
    // Update servo states from MAVLink data
    uint16_t servo_values[16] = {
        servo_raw.servo1_raw, servo_raw.servo2_raw, servo_raw.servo3_raw, servo_raw.servo4_raw,
        servo_raw.servo5_raw, servo_raw.servo6_raw, servo_raw.servo7_raw, servo_raw.servo8_raw,
        servo_raw.servo9_raw, servo_raw.servo10_raw, servo_raw.servo11_raw, servo_raw.servo12_raw,
        servo_raw.servo13_raw, servo_raw.servo14_raw, servo_raw.servo15_raw, servo_raw.servo16_raw
    };
    
    // 各サーボの値を処理
    for (size_t i = 0; i < 16 && i < servo_states_.size(); i++) {
        uint16_t old_pulse = servo_states_[i].pulse_us;
        bool old_enabled = servo_states_[i].enabled;

        if (servo_values[i] == 0) {
            // Only mark as initialized if we previously had a non-zero value
            // This prevents phantom servos from being marked as active
            if (servo_states_[i].initialized) {
                servo_states_[i].enabled = false;
                servo_states_[i].status = 1; // NOT_INITIALIZED
                servo_states_[i].current_angle_deg = 0.0f;
            }
            // Don't set initialized = false here to keep existing devices
        } else if (servo_values[i] >= 500 && servo_values[i] <= 2500) {
            servo_states_[i].initialized = true;  // Mark as initialized
            servo_states_[i].enabled = true;
            servo_states_[i].status = 0; // OK
            servo_states_[i].current_angle_deg = ((servo_values[i] - 500.0f) / 1500.0f) * 120.0f - 60.0f;
        } else {
            servo_states_[i].initialized = true;  // Mark as initialized even if error
            servo_states_[i].enabled = false;
            servo_states_[i].status = 2; // ERROR
            servo_states_[i].current_angle_deg = 0.0f;
        }
        
        // 状態が変化したらログ出力
        if (i < 3 && (old_pulse != servo_states_[i].pulse_us || old_enabled != servo_states_[i].enabled)) {
            RCLCPP_INFO(node_->get_logger(), 
                "Servo %zu state changed: pulse=%d->%d, enabled=%d->%d, angle=%.1f°",
                i+1, old_pulse, servo_states_[i].pulse_us, 
                old_enabled, servo_states_[i].enabled,
                servo_states_[i].current_angle_deg);
        }
    }
    
    // 必ずpublish
    publishServoStates();
    RCLCPP_INFO_ONCE(node_->get_logger(), "Publishing servo states to /servo/states topic");
}

void ServoController::handleManualControl(const mavlink_manual_control_t& manual) {
    std::lock_guard<std::mutex> lock(servo_mutex_);
    
    // Map manual control to first 4 servos (X, Y, Z, R axes)
    if (servo_states_.size() > 0) {
        servo_states_[0].target_angle_deg = (manual.x / 1000.0f) * 60.0f;
    }
    if (servo_states_.size() > 1) {
        servo_states_[1].target_angle_deg = (manual.y / 1000.0f) * 60.0f;
    }
    if (servo_states_.size() > 2) {
        servo_states_[2].target_angle_deg = (manual.z / 1000.0f) * 60.0f;
    }
    if (servo_states_.size() > 3) {
        servo_states_[3].target_angle_deg = (manual.r / 1000.0f) * 60.0f;
    }
}

void ServoController::handleRCChannelsOverride(const mavlink_rc_channels_override_t& rc_override) {
    std::lock_guard<std::mutex> lock(servo_mutex_);
    
    uint16_t channels[8] = {
        rc_override.chan1_raw, rc_override.chan2_raw, rc_override.chan3_raw, rc_override.chan4_raw,
        rc_override.chan5_raw, rc_override.chan6_raw, rc_override.chan7_raw, rc_override.chan8_raw
    };
    
    for (size_t i = 0; i < 8 && i < servo_states_.size(); i++) {
        if (channels[i] != UINT16_MAX && channels[i] <= 2000) {
            servo_states_[i].pulse_us = channels[i];
            servo_states_[i].target_angle_deg = ((channels[i] - 500.0f) / 1500.0f) * 120.0f - 60.0f;
        }
    }
}

bool ServoController::getManualControlMessage(mavlink_message_t& msg, uint8_t system_id, 
                                             uint8_t component_id, uint8_t target_system) {
    std::lock_guard<std::mutex> lock(servo_mutex_);
    
    int16_t x = 0, y = 0, z = 0, r = 0;
    
    if (servo_states_.size() > 0) {
        x = static_cast<int16_t>((servo_states_[0].target_angle_deg / 60.0f) * 1000.0f);
    }
    if (servo_states_.size() > 1) {
        y = static_cast<int16_t>((servo_states_[1].target_angle_deg / 60.0f) * 1000.0f);
    }
    if (servo_states_.size() > 2) {
        z = static_cast<int16_t>((servo_states_[2].target_angle_deg / 60.0f) * 1000.0f);
    }
    if (servo_states_.size() > 3) {
        r = static_cast<int16_t>((servo_states_[3].target_angle_deg / 60.0f) * 1000.0f);
    }
    
    mavlink_msg_manual_control_pack(system_id, component_id, &msg,
                                   target_system, x, y, z, r, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    return true;
}

bool ServoController::getRCOverrideMessage(mavlink_message_t& msg, uint8_t system_id,
                                          uint8_t component_id, uint8_t target_system) {
    std::lock_guard<std::mutex> lock(servo_mutex_);
    
    uint16_t channels[8] = {UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX,
                           UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX};
    
    bool has_command = false;
    for (size_t i = 0; i < 8 && i < servo_states_.size(); i++) {
        if (servo_states_[i].initialized && servo_states_[i].enabled) {
            // Always send the exact user-commanded pulse value to maintain watchdog
            uint16_t pulse_to_send = servo_states_[i].pulse_us;

            if (pulse_to_send == 0) {
                // Default to center position if no pulse is set (0° = 1250μs)
                pulse_to_send = 1250;
            }

            channels[i] = pulse_to_send;
            has_command = true;

            // Debug logging every 50 cycles (reduce spam)
            static int debug_counter = 0;
            if (debug_counter++ % 50 == 0) {
                std::cout << "Servo " << (1) << ": sending PWM=" << pulse_to_send
                         << "μs, target_angle=" << servo_states_[0].target_angle_deg << "°" << std::endl;
            }
        }
    }

    if (!has_command) {
        return false;
    }
    
    mavlink_msg_rc_channels_override_pack(system_id, component_id, &msg,
                                         target_system, MAV_COMP_ID_ALL,
                                         channels[0], channels[1], channels[2], channels[3],
                                         channels[4], channels[5], channels[6], channels[7],
                                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    return true;
}

void ServoController::servoCommandCallback(const stm32_mavlink_msgs::msg::ServoCommand::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(servo_mutex_);

    if (msg->servo_id > 0 && msg->servo_id <= servo_states_.size()) {
        size_t idx = msg->servo_id - 1;

        // Mark servo as initialized when it receives a command
        servo_states_[idx].initialized = true;

        if (msg->pulse_us > 0) {
            // Direct PWM control - preserve exact user value
            servo_states_[idx].pulse_us = msg->pulse_us;
            // Convert PWM back to angle: 500μs = -60°, 1250μs = 0°, 2000μs = +60°
            servo_states_[idx].target_angle_deg = ((msg->pulse_us - 500.0f) / 1500.0f) * 120.0f - 60.0f;
        } else {
            // Angle control - convert to PWM preserving user's exact angle intent
            servo_states_[idx].target_angle_deg = msg->angle_deg;

            // Convert angle to PWM for 120° servo (-60° to +60°)
            // New mapping: -60° = 500μs, 0° = 1250μs, +60° = 2000μs
            // PWM = ((angle + 60) / 120) * 1500 + 500
            servo_states_[idx].pulse_us = static_cast<uint16_t>(((msg->angle_deg + 60.0f) / 120.0f) * 1500.0f + 500.0f);

            // Ensure PWM stays in valid range (500-2000μs)
            if (servo_states_[idx].pulse_us < 500) servo_states_[idx].pulse_us = 500;
            if (servo_states_[idx].pulse_us > 2000) servo_states_[idx].pulse_us = 2000;
        }

        servo_states_[idx].enabled = msg->enable;
    }
}

void ServoController::servoConfigCallback(
    const std::shared_ptr<stm32_mavlink_msgs::srv::SetServoConfig::Request> request,
    std::shared_ptr<stm32_mavlink_msgs::srv::SetServoConfig::Response> response) {
    
    // In a real implementation, this would send configuration to STM32
    // For now, just acknowledge
    response->success = true;
    response->message = "Configuration request received";
    
    RCLCPP_INFO(node_->get_logger(), "Servo %d config request received", request->servo_id);
}

void ServoController::publishServoStates() {
    auto now = node_->now();

    for (size_t i = 0; i < servo_states_.size(); i++) {
        // Only publish servo states for servos that have been initialized
        // This prevents phantom servos (IDs 5-16) from appearing in MAVLink Wizard
        if (!servo_states_[i].initialized) {
            continue;
        }

        auto msg = stm32_mavlink_msgs::msg::ServoState();
        msg.header.stamp = now;
        msg.servo_id = i + 1;
        msg.current_angle_deg = servo_states_[i].current_angle_deg;
        msg.target_angle_deg = servo_states_[i].target_angle_deg;
        msg.pulse_us = servo_states_[i].pulse_us;
        msg.enabled = servo_states_[i].enabled;
        msg.status = servo_states_[i].status;
        msg.error_count = 0;

        servo_state_pub_->publish(msg);
    }
}

} // namespace stm32_mavlink_uart
