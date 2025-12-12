#include "rs485_motor_interface/rs485_motor_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace rs485_motor_interface
{

RS485MotorNode::RS485MotorNode()
: Node("rs485_motor_node"),
  polling_rate_hz_(10.0),
  retry_count_(3),
  timeout_ms_(100)
{
  // Declare parameters
  this->declare_parameter<double>("polling_rate_hz", 10.0);
  this->declare_parameter<int>("retry_count", 3);
  this->declare_parameter<int>("timeout_ms", 100);

  // Declare motor configuration parameters
  this->declare_parameter<std::vector<int64_t>>("mavlink_ids", std::vector<int64_t>{});
  this->declare_parameter<std::vector<int64_t>>("device_ids", std::vector<int64_t>{});
  this->declare_parameter<std::vector<int64_t>>("motor_indices", std::vector<int64_t>{});
  this->declare_parameter<std::vector<std::string>>("protocol_types", std::vector<std::string>{});

  // Get parameters
  this->get_parameter("polling_rate_hz", polling_rate_hz_);
  this->get_parameter("retry_count", retry_count_);
  this->get_parameter("timeout_ms", timeout_ms_);

  // Load motor configurations
  loadMotorConfigurations();

  // Create subscribers
  motor_cmd_sub_ = this->create_subscription<stm32_mavlink_msgs::msg::DCMotorCommand>(
    "/dcmotor/tx",
    10,
    std::bind(&RS485MotorNode::motorCommandCallback, this, std::placeholders::_1)
  );

  rs485_read_resp_sub_ = this->create_subscription<stm32_mavlink_msgs::msg::RS485ReadResponse>(
    "/rs485/read_resp",
    10,
    std::bind(&RS485MotorNode::rs485ReadResponseCallback, this, std::placeholders::_1)
  );

  // Create publishers
  motor_state_pub_ = this->create_publisher<stm32_mavlink_msgs::msg::DCMotorState>(
    "/dcmotor/rx",
    10
  );

  rs485_write_req_pub_ = this->create_publisher<stm32_mavlink_msgs::msg::RS485WriteRequest>(
    "/rs485/write_req",
    10
  );

  rs485_read_req_pub_ = this->create_publisher<stm32_mavlink_msgs::msg::RS485ReadRequest>(
    "/rs485/read_req",
    10
  );

  diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics",
    10
  );

  // Create polling timer
  auto polling_period = std::chrono::duration<double>(1.0 / polling_rate_hz_);
  polling_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(polling_period),
    std::bind(&RS485MotorNode::pollingTimerCallback, this)
  );

  RCLCPP_INFO(this->get_logger(), "RS485 Motor Interface Node started");
  RCLCPP_INFO(this->get_logger(), "Configured %zu motors", motor_configs_.size());
  RCLCPP_INFO(this->get_logger(), "Polling rate: %.1f Hz", polling_rate_hz_);
}

void RS485MotorNode::loadMotorConfigurations()
{
  // Get parameter arrays
  std::vector<int64_t> mavlink_ids;
  std::vector<int64_t> device_ids;
  std::vector<int64_t> motor_indices;
  std::vector<std::string> protocol_types;

  this->get_parameter("mavlink_ids", mavlink_ids);
  this->get_parameter("device_ids", device_ids);
  this->get_parameter("motor_indices", motor_indices);
  this->get_parameter("protocol_types", protocol_types);

  // Validate array sizes
  size_t num_motors = mavlink_ids.size();
  if (device_ids.size() != num_motors ||
      motor_indices.size() != num_motors ||
      protocol_types.size() != num_motors) {
    RCLCPP_ERROR(this->get_logger(), "Motor configuration arrays have mismatched sizes!");
    return;
  }

  // Create motor configurations
  for (size_t i = 0; i < num_motors; ++i) {
    MotorConfig config;
    config.motor_id = static_cast<uint8_t>(mavlink_ids[i]);
    config.device_id = static_cast<uint8_t>(device_ids[i]);
    config.motor_index = static_cast<uint8_t>(motor_indices[i]);
    config.bus_address = config.device_id;  // bus_address = device_id for v2.2
    config.protocol_type = protocol_types[i];

    // Default values - will be auto-detected from motor via RS485 during runtime
    config.encoder_resolution = 4096;  // Default, can be read from REG_ENCODER_RESOLUTION
    config.max_current = 10.0;         // Default, can be read from REG_MAX_CURRENT
    config.enabled = false;            // Start disabled

    // Validate motor ID range (30-49)
    if (config.motor_id < 30 || config.motor_id > 49) {
      RCLCPP_WARN(this->get_logger(), "MAVLink motor ID %d is outside valid range (30-49), skipping",
                  config.motor_id);
      continue;
    }

    // Validate device_id (1-15)
    if (config.device_id < 1 || config.device_id > 15) {
      RCLCPP_WARN(this->get_logger(), "Device ID %d is outside valid range (1-15), skipping",
                  config.device_id);
      continue;
    }

    // Validate motor_index (0-2)
    if (config.motor_index > 2) {
      RCLCPP_WARN(this->get_logger(), "Motor index %d is outside valid range (0-2), skipping",
                  config.motor_index);
      continue;
    }

    // Create protocol instance if not already created
    if (protocols_.find(config.protocol_type) == protocols_.end()) {
      auto protocol = createProtocol(config.protocol_type);
      if (!protocol) {
        RCLCPP_ERROR(this->get_logger(), "Unknown protocol type: %s", config.protocol_type.c_str());
        continue;
      }
      protocols_[config.protocol_type] = std::move(protocol);
      RCLCPP_INFO(this->get_logger(), "Created protocol instance: %s", config.protocol_type.c_str());
    }

    // Set encoder resolution and motor mapping in protocol
    if (auto* protocol = protocols_[config.protocol_type].get()) {
      protocol->setEncoderResolution(config.motor_id, config.encoder_resolution);
      protocol->setMotorMapping(config.motor_id, config.device_id, config.motor_index);
    }

    motor_configs_[config.motor_id] = config;

    RCLCPP_INFO(this->get_logger(),
                "Configured MAVLink ID %d -> Device %d, Motor %d (bus_addr=%d, protocol=%s)",
                config.motor_id, config.device_id, config.motor_index,
                config.bus_address, config.protocol_type.c_str());
  }

  if (num_motors > 0) {
    RCLCPP_INFO(this->get_logger(),
                "Motor parameters (encoder resolution, max current) will be auto-detected via RS485");
  }
}

void RS485MotorNode::motorCommandCallback(
  const stm32_mavlink_msgs::msg::DCMotorCommand::SharedPtr msg)
{
  // Check if motor is configured
  auto* config = getMotorConfig(msg->motor_id);
  if (!config) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Received command for unconfigured motor %d", msg->motor_id);
    return;
  }

  // Get protocol
  auto* protocol = getProtocol(msg->motor_id);
  if (!protocol) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "No protocol found for motor %d", msg->motor_id);
    return;
  }

  // Convert command to RS485 write requests
  auto write_requests = protocol->commandToWrites(*msg);

  // Publish write requests
  for (const auto& req : write_requests) {
    rs485_write_req_pub_->publish(req);
  }

  // Update statistics
  command_count_[msg->motor_id]++;

  RCLCPP_DEBUG(this->get_logger(), "Sent %zu write requests for motor %d",
               write_requests.size(), msg->motor_id);
}

void RS485MotorNode::rs485ReadResponseCallback(
  const stm32_mavlink_msgs::msg::RS485ReadResponse::SharedPtr msg)
{
  // Check if motor is configured
  auto* config = getMotorConfig(msg->motor_id);
  if (!config) {
    return;  // Ignore responses for unconfigured motors
  }

  // Get protocol
  auto* protocol = getProtocol(msg->motor_id);
  if (!protocol) {
    return;
  }

  // Parse response into motor state
  auto state = protocol->parseReadResponse(*msg, msg->motor_id);

  // Publish motor state
  motor_state_pub_->publish(state);

  // Update statistics
  response_count_[msg->motor_id]++;
  last_response_time_[msg->motor_id] = this->now();

  RCLCPP_DEBUG(this->get_logger(), "Received state for motor %d: pos=%.3f rad, vel=%.3f rad/s",
               msg->motor_id, state.position_rad, state.velocity_rad_s);
}

void RS485MotorNode::pollingTimerCallback()
{
  // Poll each configured motor
  for (const auto& [motor_id, config] : motor_configs_) {
    auto* protocol = getProtocol(motor_id);
    if (!protocol) {
      continue;
    }

    // Create state polling requests
    auto read_requests = protocol->createStatePollingRequests(motor_id, config.bus_address);

    // Publish read requests
    for (const auto& req : read_requests) {
      rs485_read_req_pub_->publish(req);
    }

    // Check for timeouts
    auto it = last_response_time_.find(motor_id);
    if (it != last_response_time_.end()) {
      auto elapsed = (this->now() - it->second).seconds();
      if (elapsed > (timeout_ms_ / 1000.0)) {
        timeout_count_[motor_id]++;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Motor %d timeout: no response for %.1f seconds",
                             motor_id, elapsed);
      }
    }
  }

  // Publish diagnostics periodically (every 10 polls)
  static int poll_count = 0;
  if (++poll_count >= 10) {
    publishDiagnostics();
    poll_count = 0;
  }
}

void RS485MotorNode::publishDiagnostics()
{
  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = this->now();

  for (const auto& [motor_id, config] : motor_configs_) {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "RS485 Motor " + std::to_string(motor_id);
    status.hardware_id = "motor_" + std::to_string(motor_id);

    // Determine status level
    auto timeout_it = timeout_count_.find(motor_id);
    auto response_it = response_count_.find(motor_id);

    if (response_it == response_count_.end() || response_it->second == 0) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "No responses received";
    } else if (timeout_it != timeout_count_.end() && timeout_it->second > 0) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "Communication timeouts detected";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "Operating normally";
    }

    // Add key-value pairs
    diagnostic_msgs::msg::KeyValue kv;

    kv.key = "motor_id";
    kv.value = std::to_string(motor_id);
    status.values.push_back(kv);

    kv.key = "bus_address";
    kv.value = std::to_string(config.bus_address);
    status.values.push_back(kv);

    kv.key = "protocol";
    kv.value = config.protocol_type;
    status.values.push_back(kv);

    auto cmd_it = command_count_.find(motor_id);
    kv.key = "commands_sent";
    kv.value = std::to_string(cmd_it != command_count_.end() ? cmd_it->second : 0);
    status.values.push_back(kv);

    kv.key = "responses_received";
    kv.value = std::to_string(response_it != response_count_.end() ? response_it->second : 0);
    status.values.push_back(kv);

    kv.key = "timeouts";
    kv.value = std::to_string(timeout_it != timeout_count_.end() ? timeout_it->second : 0);
    status.values.push_back(kv);

    diag_array.status.push_back(status);
  }

  diagnostics_pub_->publish(diag_array);
}

MotorProtocol* RS485MotorNode::getProtocol(uint8_t motor_id)
{
  auto* config = getMotorConfig(motor_id);
  if (!config) {
    return nullptr;
  }

  auto it = protocols_.find(config->protocol_type);
  if (it != protocols_.end()) {
    return it->second.get();
  }

  return nullptr;
}

const MotorConfig* RS485MotorNode::getMotorConfig(uint8_t motor_id) const
{
  auto it = motor_configs_.find(motor_id);
  if (it != motor_configs_.end()) {
    return &it->second;
  }
  return nullptr;
}

}  // namespace rs485_motor_interface
