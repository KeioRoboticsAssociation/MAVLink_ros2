#ifndef RS485_MOTOR_INTERFACE__RS485_MOTOR_NODE_HPP_
#define RS485_MOTOR_INTERFACE__RS485_MOTOR_NODE_HPP_

#include <memory>
#include <map>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "stm32_mavlink_msgs/msg/dc_motor_command.hpp"
#include "stm32_mavlink_msgs/msg/dc_motor_state.hpp"
#include "stm32_mavlink_msgs/msg/rs485_write_request.hpp"
#include "stm32_mavlink_msgs/msg/rs485_read_response.hpp"
#include "stm32_mavlink_msgs/msg/rs485_read_request.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include "rs485_motor_interface/motor_protocol.hpp"

namespace rs485_motor_interface
{

/**
 * @brief Motor configuration structure
 */
struct MotorConfig {
  uint8_t motor_id;           // Motor ID (30-49) - ROS2/MAVLink motor ID
  uint8_t device_id;          // RS485 device ID (1-15) - physical board address
  uint8_t motor_index;        // Motor index on device (0-2)
  uint8_t bus_address;        // RS485 bus address (same as device_id for v2.2)
  std::string protocol_type;  // Protocol name (e.g., "ikeya_md")
  uint32_t encoder_resolution;// Encoder counts per revolution
  double max_current;         // Maximum current (A)
  bool enabled;               // Motor enabled by default
};

/**
 * @brief High-level interface node for RS485 motors
 *
 * This node provides a simplified DCMotor interface for RS485 motors,
 * translating between high-level commands and low-level RS485 register operations.
 *
 * Topics:
 *   Subscribed:
 *     /dcmotor/tx (DCMotorCommand) - Motor commands from user applications
 *     /rs485/read_resp (RS485ReadResponse) - Read responses from STM32
 *
 *   Published:
 *     /dcmotor/rx (DCMotorState) - Motor state feedback to user applications
 *     /rs485/write_req (RS485WriteRequest) - Write requests to STM32
 *     /rs485/read_req (RS485ReadRequest) - Read requests to STM32
 *     /diagnostics (DiagnosticArray) - System diagnostics
 */
class RS485MotorNode : public rclcpp::Node
{
public:
  RS485MotorNode();
  ~RS485MotorNode() override = default;

private:
  /**
   * @brief Load motor configurations from parameters
   */
  void loadMotorConfigurations();

  /**
   * @brief Callback for DCMotor command messages
   */
  void motorCommandCallback(const stm32_mavlink_msgs::msg::DCMotorCommand::SharedPtr msg);

  /**
   * @brief Callback for RS485 read response messages
   */
  void rs485ReadResponseCallback(const stm32_mavlink_msgs::msg::RS485ReadResponse::SharedPtr msg);

  /**
   * @brief Timer callback for periodic state polling
   */
  void pollingTimerCallback();

  /**
   * @brief Publish diagnostics information
   */
  void publishDiagnostics();

  /**
   * @brief Get protocol instance for a motor
   * @param motor_id Motor ID
   * @return Pointer to protocol instance, or nullptr if not found
   */
  MotorProtocol* getProtocol(uint8_t motor_id);

  /**
   * @brief Get motor configuration
   * @param motor_id Motor ID
   * @return Pointer to motor configuration, or nullptr if not found
   */
  const MotorConfig* getMotorConfig(uint8_t motor_id) const;

  // ROS2 subscribers
  rclcpp::Subscription<stm32_mavlink_msgs::msg::DCMotorCommand>::SharedPtr motor_cmd_sub_;
  rclcpp::Subscription<stm32_mavlink_msgs::msg::RS485ReadResponse>::SharedPtr rs485_read_resp_sub_;

  // ROS2 publishers
  rclcpp::Publisher<stm32_mavlink_msgs::msg::DCMotorState>::SharedPtr motor_state_pub_;
  rclcpp::Publisher<stm32_mavlink_msgs::msg::RS485WriteRequest>::SharedPtr rs485_write_req_pub_;
  rclcpp::Publisher<stm32_mavlink_msgs::msg::RS485ReadRequest>::SharedPtr rs485_read_req_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  // Timer for state polling
  rclcpp::TimerBase::SharedPtr polling_timer_;

  // Motor configurations (motor_id -> config)
  std::map<uint8_t, MotorConfig> motor_configs_;

  // Protocol instances (protocol_name -> protocol)
  std::map<std::string, std::unique_ptr<MotorProtocol>> protocols_;

  // Parameters
  double polling_rate_hz_;           // State polling rate (default 10Hz)
  int retry_count_;                  // Retry count for failed operations (default 3)
  int timeout_ms_;                   // Timeout for RS485 operations (default 100ms)

  // Statistics
  std::map<uint8_t, size_t> command_count_;       // Commands sent per motor
  std::map<uint8_t, size_t> response_count_;      // Responses received per motor
  std::map<uint8_t, size_t> timeout_count_;       // Timeouts per motor
  std::map<uint8_t, rclcpp::Time> last_response_time_;  // Last response time per motor
};

}  // namespace rs485_motor_interface

#endif  // RS485_MOTOR_INTERFACE__RS485_MOTOR_NODE_HPP_
