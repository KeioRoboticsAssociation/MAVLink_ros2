#include "stm32_mavlink_udp/rs485motor_controller.hpp"
#include <chrono>

namespace stm32_mavlink_udp {

RS485MotorController::RS485MotorController(rclcpp::Node* node)
    : node_(node) {

    // Create ROS2 publishers
    motor_state_pub_ = node_->create_publisher<msg::RS485MotorState>("/rs485motor/state", 10);
    read_response_pub_ = node_->create_publisher<msg::RS485ReadResponse>("/rs485motor/read_response", 10);
    write_response_pub_ = node_->create_publisher<msg::RS485WriteResponse>("/rs485motor/write_response", 10);

    // Create ROS2 subscribers
    motor_cmd_sub_ = node_->create_subscription<msg::RS485MotorCommand>(
        "/rs485motor/command", 10,
        std::bind(&RS485MotorController::motorCommandCallback, this, std::placeholders::_1));
    read_request_sub_ = node_->create_subscription<msg::RS485ReadRequest>(
        "/rs485motor/read_request", 10,
        std::bind(&RS485MotorController::readRequestCallback, this, std::placeholders::_1));
    write_request_sub_ = node_->create_subscription<msg::RS485WriteRequest>(
        "/rs485motor/write_request", 10,
        std::bind(&RS485MotorController::writeRequestCallback, this, std::placeholders::_1));
    flash_save_request_sub_ = node_->create_subscription<msg::RS485FlashSaveRequest>(
        "/rs485motor/flash_save_request", 10,
        std::bind(&RS485MotorController::flashSaveRequestCallback, this, std::placeholders::_1));

    // Create ROS2 services
    read_param_service_ = node_->create_service<srv::ReadRS485Param>(
        "/rs485motor/read_param",
        std::bind(&RS485MotorController::readParamService, this, std::placeholders::_1, std::placeholders::_2));
    write_param_service_ = node_->create_service<srv::WriteRS485Param>(
        "/rs485motor/write_param",
        std::bind(&RS485MotorController::writeParamService, this, std::placeholders::_1, std::placeholders::_2));
    flash_save_service_ = node_->create_service<srv::FlashSaveRS485>(
        "/rs485motor/flash_save",
        std::bind(&RS485MotorController::flashSaveService, this, std::placeholders::_1, std::placeholders::_2));

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
    RCLCPP_INFO(node_->get_logger(), "  Topics: /rs485motor/state, /rs485motor/command");
    RCLCPP_INFO(node_->get_logger(), "  Read: /rs485motor/read_request, /rs485motor/read_response");
    RCLCPP_INFO(node_->get_logger(), "  Write: /rs485motor/write_request, /rs485motor/write_response");
    RCLCPP_INFO(node_->get_logger(), "  Flash: /rs485motor/flash_save_request");
    RCLCPP_INFO(node_->get_logger(), "  Services: /rs485motor/read_param, /rs485motor/write_param, /rs485motor/flash_save");
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

    // Check for read request timeouts (1 second timeout)
    std::vector<uint32_t> timed_out_reads;
    for (auto& [key, pending_read] : pending_reads_) {
        auto elapsed = (now - pending_read.request_time).seconds();
        if (elapsed > 1.0) {
            pending_read.response->success = false;
            pending_read.response->status = 3; // TIMEOUT
            pending_read.response->message = "Read request timeout";
            timed_out_reads.push_back(key);
        }
    }
    for (auto key : timed_out_reads) {
        pending_reads_.erase(key);
    }

    // Check for write request timeouts (1 second timeout)
    std::vector<uint32_t> timed_out_writes;
    for (auto& [key, pending_write] : pending_writes_) {
        auto elapsed = (now - pending_write.request_time).seconds();
        if (elapsed > 1.0) {
            pending_write.response->success = false;
            pending_write.response->status = 3; // TIMEOUT
            pending_write.response->message = "Write request timeout";
            timed_out_writes.push_back(key);
        }
    }
    for (auto key : timed_out_writes) {
        pending_writes_.erase(key);
    }

    // Check for flash save timeouts (2 second timeout - flash save takes longer)
    std::vector<uint8_t> timed_out_flash_saves;
    for (auto& [motor_id, pending_flash] : pending_flash_saves_) {
        auto elapsed = (now - pending_flash.request_time).seconds();
        if (elapsed > 2.0) {
            pending_flash.response->success = false;
            pending_flash.response->result = 4; // MAV_RESULT_FAILED
            pending_flash.response->message = "Flash save timeout";
            timed_out_flash_saves.push_back(motor_id);
        }
    }
    for (auto motor_id : timed_out_flash_saves) {
        pending_flash_saves_.erase(motor_id);
    }
}

void RS485MotorController::readRequestCallback(const msg::RS485ReadRequest::SharedPtr msg) {
    // Validate motor ID
    if (msg->motor_id < RS485_MOTOR_ID_MIN || msg->motor_id > RS485_MOTOR_ID_MAX) {
        RCLCPP_WARN(node_->get_logger(), "Read request for invalid motor ID %d (valid: %d-%d)",
                   msg->motor_id, RS485_MOTOR_ID_MIN, RS485_MOTOR_ID_MAX);
        return;
    }

    // Validate length
    if (msg->length < 1 || msg->length > 64) {
        RCLCPP_WARN(node_->get_logger(), "Invalid read length %d (valid: 1-64)", msg->length);
        return;
    }

    RCLCPP_DEBUG(node_->get_logger(), "RS485 read request: motor_id=%d, address=0x%04X, length=%d",
                msg->motor_id, msg->address, msg->length);

    // Add to pending read requests queue
    pending_read_requests_.push(*msg);
}

void RS485MotorController::readParamService(const std::shared_ptr<srv::ReadRS485Param::Request> request,
                                            std::shared_ptr<srv::ReadRS485Param::Response> response) {
    // Validate motor ID
    if (request->motor_id < RS485_MOTOR_ID_MIN || request->motor_id > RS485_MOTOR_ID_MAX) {
        response->success = false;
        response->status = 5; // ERROR_INVALID_PARAMETER
        response->message = "Invalid motor ID";
        return;
    }

    // Validate length
    if (request->length < 1 || request->length > 64) {
        response->success = false;
        response->status = 5; // ERROR_INVALID_PARAMETER
        response->message = "Invalid read length (must be 1-64)";
        return;
    }

    // Create read request message
    msg::RS485ReadRequest read_req;
    read_req.motor_id = request->motor_id;
    read_req.address = request->address;
    read_req.length = request->length;

    // Generate unique key for this request
    uint32_t key = (static_cast<uint32_t>(request->motor_id) << 16) | request->address;

    // Store pending read with response callback
    PendingRead pending;
    pending.response = response;
    pending.request_time = node_->now();
    pending_reads_[key] = pending;

    // Add to queue for MAVLink transmission
    pending_read_requests_.push(read_req);

    RCLCPP_INFO(node_->get_logger(), "RS485 read service: motor_id=%d, address=0x%04X, length=%d",
               request->motor_id, request->address, request->length);
}

bool RS485MotorController::getReadRequestMessage(mavlink_message_t& msg, uint8_t system_id,
                                                   uint8_t component_id, uint8_t target_system_id) {
    if (pending_read_requests_.empty()) {
        return false;
    }

    auto req = pending_read_requests_.front();
    pending_read_requests_.pop();

    // Pack RS485 read request message (MAVLink message ID 12007)
    mavlink_msg_rs485_read_request_pack(system_id, component_id, &msg,
                                        req.motor_id, req.address, req.length);

    RCLCPP_DEBUG(node_->get_logger(), "Sending RS485_READ_REQUEST: motor_id=%d, address=0x%04X, length=%d",
                req.motor_id, req.address, req.length);

    return true;
}

void RS485MotorController::handleReadResponse(const mavlink_message_t& msg) {
    // Decode RS485 read response (MAVLink message ID 12008)
    mavlink_rs485_read_response_t mavlink_response;
    mavlink_msg_rs485_read_response_decode(&msg, &mavlink_response);

    // Create ROS2 message
    msg::RS485ReadResponse response_msg;
    response_msg.motor_id = mavlink_response.motor_id;
    response_msg.address = mavlink_response.address;
    response_msg.length = mavlink_response.length;
    response_msg.status = mavlink_response.status;
    response_msg.rs485_error = mavlink_response.rs485_error;

    // Copy data (ensure we don't overflow)
    for (int i = 0; i < std::min(static_cast<int>(mavlink_response.length), 64); i++) {
        response_msg.data[i] = mavlink_response.data[i];
    }

    // Publish response message
    read_response_pub_->publish(response_msg);

    RCLCPP_DEBUG(node_->get_logger(), "RS485_READ_RESPONSE: motor_id=%d, address=0x%04X, length=%d, status=%d, rs485_error=%d",
                response_msg.motor_id, response_msg.address, response_msg.length,
                response_msg.status, response_msg.rs485_error);

    // Check if this is a response to a service request
    uint32_t key = (static_cast<uint32_t>(mavlink_response.motor_id) << 16) | mavlink_response.address;
    auto it = pending_reads_.find(key);
    if (it != pending_reads_.end()) {
        // Fill in service response
        auto& srv_response = it->second.response;
        srv_response->success = (mavlink_response.status == 0);
        srv_response->status = mavlink_response.status;
        srv_response->rs485_error = mavlink_response.rs485_error;

        // Copy data
        for (int i = 0; i < std::min(static_cast<int>(mavlink_response.length), 64); i++) {
            srv_response->data[i] = mavlink_response.data[i];
        }

        // Set status message
        if (srv_response->success) {
            srv_response->message = "Read successful";
        } else {
            srv_response->message = "Read failed: status=" + std::to_string(mavlink_response.status) +
                                   ", rs485_error=" + std::to_string(mavlink_response.rs485_error);
        }

        // Remove from pending map
        pending_reads_.erase(it);

        RCLCPP_INFO(node_->get_logger(), "RS485 read service completed: motor_id=%d, address=0x%04X, success=%s",
                   mavlink_response.motor_id, mavlink_response.address,
                   srv_response->success ? "true" : "false");
    }
}

void RS485MotorController::writeRequestCallback(const msg::RS485WriteRequest::SharedPtr msg) {
    // Validate motor ID
    if (msg->motor_id < RS485_MOTOR_ID_MIN || msg->motor_id > RS485_MOTOR_ID_MAX) {
        RCLCPP_WARN(node_->get_logger(), "Write request for invalid motor ID %d (valid: %d-%d)",
                   msg->motor_id, RS485_MOTOR_ID_MIN, RS485_MOTOR_ID_MAX);
        return;
    }

    // Validate length
    if (msg->length < 1 || msg->length > 64) {
        RCLCPP_WARN(node_->get_logger(), "Invalid write length %d (valid: 1-64)", msg->length);
        return;
    }

    RCLCPP_DEBUG(node_->get_logger(), "RS485 write request: motor_id=%d, address=0x%04X, length=%d",
                msg->motor_id, msg->address, msg->length);

    // Add to pending write requests queue
    pending_write_requests_.push(*msg);
}

void RS485MotorController::flashSaveRequestCallback(const msg::RS485FlashSaveRequest::SharedPtr msg) {
    // Validate motor ID
    if (msg->motor_id < RS485_MOTOR_ID_MIN || msg->motor_id > RS485_MOTOR_ID_MAX) {
        RCLCPP_WARN(node_->get_logger(), "Flash save request for invalid motor ID %d (valid: %d-%d)",
                   msg->motor_id, RS485_MOTOR_ID_MIN, RS485_MOTOR_ID_MAX);
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "RS485 flash save request: motor_id=%d", msg->motor_id);

    // Add to pending flash save requests queue
    pending_flash_save_requests_.push(*msg);
}

void RS485MotorController::writeParamService(const std::shared_ptr<srv::WriteRS485Param::Request> request,
                                             std::shared_ptr<srv::WriteRS485Param::Response> response) {
    // Validate motor ID
    if (request->motor_id < RS485_MOTOR_ID_MIN || request->motor_id > RS485_MOTOR_ID_MAX) {
        response->success = false;
        response->status = 5; // ERROR_INVALID_PARAMETER
        response->message = "Invalid motor ID";
        return;
    }

    // Validate length
    if (request->length < 1 || request->length > 64) {
        response->success = false;
        response->status = 5; // ERROR_INVALID_PARAMETER
        response->message = "Invalid write length (must be 1-64)";
        return;
    }

    // Create write request message
    msg::RS485WriteRequest write_req;
    write_req.motor_id = request->motor_id;
    write_req.address = request->address;
    write_req.length = request->length;
    write_req.data = request->data;

    // Generate unique key for this request
    uint32_t key = (static_cast<uint32_t>(request->motor_id) << 16) | request->address;

    // Store pending write with response callback
    PendingWrite pending;
    pending.response = response;
    pending.request_time = node_->now();
    pending_writes_[key] = pending;

    // Add to queue for MAVLink transmission
    pending_write_requests_.push(write_req);

    RCLCPP_INFO(node_->get_logger(), "RS485 write service: motor_id=%d, address=0x%04X, length=%d",
               request->motor_id, request->address, request->length);
}

void RS485MotorController::flashSaveService(const std::shared_ptr<srv::FlashSaveRS485::Request> request,
                                            std::shared_ptr<srv::FlashSaveRS485::Response> response) {
    // Validate motor ID
    if (request->motor_id < RS485_MOTOR_ID_MIN || request->motor_id > RS485_MOTOR_ID_MAX) {
        response->success = false;
        response->result = 4; // MAV_RESULT_FAILED
        response->message = "Invalid motor ID";
        return;
    }

    // Create flash save request
    msg::RS485FlashSaveRequest flash_req;
    flash_req.motor_id = request->motor_id;

    // Store pending flash save
    PendingFlashSave pending;
    pending.response = response;
    pending.request_time = node_->now();
    pending_flash_saves_[request->motor_id] = pending;

    // Queue for transmission
    pending_flash_save_requests_.push(flash_req);

    RCLCPP_INFO(node_->get_logger(), "RS485 flash save service: motor_id=%d", request->motor_id);
}

bool RS485MotorController::getWriteRequestMessage(mavlink_message_t& msg, uint8_t system_id,
                                                   uint8_t component_id, uint8_t target_system_id) {
    (void)target_system_id; // Unused parameter

    if (pending_write_requests_.empty()) {
        return false;
    }

    auto req = pending_write_requests_.front();
    pending_write_requests_.pop();

    // Pack RS485 write request message (MAVLink message ID 12009)
    uint8_t data[64];
    for (int i = 0; i < 64; i++) {
        data[i] = req.data[i];
    }

    mavlink_msg_rs485_write_request_pack(system_id, component_id, &msg,
                                         req.motor_id, req.address, req.length, data);

    RCLCPP_DEBUG(node_->get_logger(), "Sending RS485_WRITE_REQUEST: motor_id=%d, address=0x%04X, length=%d",
                req.motor_id, req.address, req.length);

    return true;
}

bool RS485MotorController::getFlashSaveRequestMessage(mavlink_message_t& msg, uint8_t system_id,
                                                       uint8_t component_id, uint8_t target_system_id) {
    (void)target_system_id; // Unused parameter

    if (pending_flash_save_requests_.empty()) {
        return false;
    }

    auto req = pending_flash_save_requests_.front();
    pending_flash_save_requests_.pop();

    // Pack RS485 flash save request message (MAVLink message ID 12011)
    mavlink_msg_rs485_flash_save_request_pack(system_id, component_id, &msg, req.motor_id);

    RCLCPP_DEBUG(node_->get_logger(), "Sending RS485_FLASH_SAVE_REQUEST: motor_id=%d", req.motor_id);

    return true;
}

void RS485MotorController::handleWriteResponse(const mavlink_message_t& msg) {
    // Decode RS485 write response (MAVLink message ID 12010)
    mavlink_rs485_write_response_t mavlink_response;
    mavlink_msg_rs485_write_response_decode(&msg, &mavlink_response);

    // Create ROS2 message
    msg::RS485WriteResponse response_msg;
    response_msg.motor_id = mavlink_response.motor_id;
    response_msg.address = mavlink_response.address;
    response_msg.length = mavlink_response.length;
    response_msg.status = mavlink_response.status;
    response_msg.rs485_error = mavlink_response.rs485_error;

    // Publish response message
    write_response_pub_->publish(response_msg);

    RCLCPP_DEBUG(node_->get_logger(), "RS485_WRITE_RESPONSE: motor_id=%d, address=0x%04X, length=%d, status=%d, rs485_error=%d",
                response_msg.motor_id, response_msg.address, response_msg.length,
                response_msg.status, response_msg.rs485_error);

    // Check if this is a response to a service request
    uint32_t key = (static_cast<uint32_t>(mavlink_response.motor_id) << 16) | mavlink_response.address;
    auto it = pending_writes_.find(key);
    if (it != pending_writes_.end()) {
        // Fill in service response
        auto& srv_response = it->second.response;
        srv_response->success = (mavlink_response.status == 0);
        srv_response->status = mavlink_response.status;
        srv_response->rs485_error = mavlink_response.rs485_error;

        // Set status message
        if (srv_response->success) {
            srv_response->message = "Write successful";
        } else {
            srv_response->message = "Write failed: status=" + std::to_string(mavlink_response.status) +
                                   ", rs485_error=" + std::to_string(mavlink_response.rs485_error);
        }

        // Remove from pending map
        pending_writes_.erase(it);

        RCLCPP_INFO(node_->get_logger(), "RS485 write service completed: motor_id=%d, address=0x%04X, success=%s",
                   mavlink_response.motor_id, mavlink_response.address,
                   srv_response->success ? "true" : "false");
    }
}

void RS485MotorController::handleCommandAck(const mavlink_message_t& msg) {
    // Decode COMMAND_ACK
    mavlink_command_ack_t ack;
    mavlink_msg_command_ack_decode(&msg, &ack);

    // Check if this is a flash save acknowledgement (command ID 12011)
    if (ack.command == 12011) {
        // Extract motor ID from result_param2 (stored in lower byte)
        uint8_t motor_id = static_cast<uint8_t>(ack.result_param2 & 0xFF);

        auto it = pending_flash_saves_.find(motor_id);
        if (it != pending_flash_saves_.end()) {
            auto& response = it->second.response;
            response->success = (ack.result == 0); // MAV_RESULT_ACCEPTED
            response->result = ack.result;
            response->error_code = static_cast<uint8_t>((ack.result_param2 >> 8) & 0xFF);

            if (response->success) {
                response->message = "Flash save successful";
            } else {
                response->message = "Flash save failed: result=" + std::to_string(ack.result) +
                                   ", error_code=" + std::to_string(response->error_code);
            }

            pending_flash_saves_.erase(it);

            RCLCPP_INFO(node_->get_logger(), "RS485 flash save completed: motor_id=%d, success=%s",
                       motor_id, response->success ? "true" : "false");
        }
    }
}

} // namespace stm32_mavlink_udp
