#ifndef RS485_MOTOR_INTERFACE__MOTOR_PROTOCOL_HPP_
#define RS485_MOTOR_INTERFACE__MOTOR_PROTOCOL_HPP_

#include <vector>
#include <memory>
#include <string>

#include "stm32_mavlink_msgs/msg/dc_motor_command.hpp"
#include "stm32_mavlink_msgs/msg/dc_motor_state.hpp"
#include "stm32_mavlink_msgs/msg/rs485_write_request.hpp"
#include "stm32_mavlink_msgs/msg/rs485_read_response.hpp"
#include "stm32_mavlink_msgs/msg/rs485_read_request.hpp"

namespace rs485_motor_interface
{

/**
 * @brief Base class for RS485 motor protocol implementations
 *
 * This abstract class defines the interface for translating between high-level
 * DCMotor messages and low-level RS485 register read/write operations.
 *
 * Subclasses implement motor-specific register maps and conversion logic.
 */
class MotorProtocol
{
public:
  virtual ~MotorProtocol() = default;

  /**
   * @brief Convert a DCMotor command to RS485 write requests
   * @param cmd High-level motor command (velocity, position, current, etc.)
   * @return Vector of RS485 write requests to send to the motor
   */
  virtual std::vector<stm32_mavlink_msgs::msg::RS485WriteRequest>
  commandToWrites(const stm32_mavlink_msgs::msg::DCMotorCommand& cmd) = 0;

  /**
   * @brief Parse an RS485 read response into a DCMotor state message
   * @param resp RS485 read response from the motor
   * @param motor_id Motor ID (30-49)
   * @return DCMotor state message with current position, velocity, current, etc.
   */
  virtual stm32_mavlink_msgs::msg::DCMotorState
  parseReadResponse(
    const stm32_mavlink_msgs::msg::RS485ReadResponse& resp,
    uint8_t motor_id) = 0;

  /**
   * @brief Create RS485 read requests for polling motor state
   * @param motor_id Motor ID (30-49)
   * @param bus_address RS485 bus address (0x01-0xFF)
   * @return Vector of RS485 read requests to query motor state
   */
  virtual std::vector<stm32_mavlink_msgs::msg::RS485ReadRequest>
  createStatePollingRequests(uint8_t motor_id, uint8_t bus_address) = 0;

  /**
   * @brief Get the protocol name
   * @return Protocol name (e.g., "ikeya_md", "generic_modbus")
   */
  virtual std::string getProtocolName() const = 0;

  /**
   * @brief Get encoder resolution for converting between counts and radians
   * @param motor_id Motor ID
   * @return Encoder counts per revolution (default 4096)
   */
  virtual uint32_t getEncoderResolution(uint8_t motor_id) const {
    (void)motor_id;  // Unused parameter
    return 4096;  // Default value
  }

  /**
   * @brief Set encoder resolution for a specific motor
   * @param motor_id Motor ID
   * @param resolution Counts per revolution
   */
  virtual void setEncoderResolution(uint8_t motor_id, uint32_t resolution) {
    (void)motor_id;
    (void)resolution;
    // Default implementation does nothing
  }

  /**
   * @brief Set motor addressing mapping for Protocol v2.2
   * @param motor_id MAVLink motor ID (30-49)
   * @param device_id RS485 device ID (1-15)
   * @param motor_index Motor index on device (0-2)
   */
  virtual void setMotorMapping(uint8_t motor_id, uint8_t device_id, uint8_t motor_index) {
    (void)motor_id;
    (void)device_id;
    (void)motor_index;
    // Default implementation does nothing
  }

protected:
  /**
   * @brief Convert radians to encoder counts
   * @param radians Angle in radians
   * @param resolution Encoder counts per revolution
   * @return Encoder counts
   */
  int32_t radiansTocounts(double radians, uint32_t resolution) const;

  /**
   * @brief Convert encoder counts to radians
   * @param counts Encoder counts
   * @param resolution Encoder counts per revolution
   * @return Angle in radians
   */
  double countsToRadians(int32_t counts, uint32_t resolution) const;

  /**
   * @brief Convert rad/s to counts/sec
   * @param rad_per_sec Angular velocity in rad/s
   * @param resolution Encoder counts per revolution
   * @return Encoder counts per second
   */
  int32_t radPerSecToCounts(double rad_per_sec, uint32_t resolution) const;

  /**
   * @brief Convert counts/sec to rad/s
   * @param counts_per_sec Encoder counts per second
   * @param resolution Encoder counts per revolution
   * @return Angular velocity in rad/s
   */
  double countsToRadPerSec(int32_t counts_per_sec, uint32_t resolution) const;
};

/**
 * @brief Factory function to create protocol instances
 * @param protocol_name Protocol name (e.g., "ikeya_md")
 * @return Unique pointer to protocol instance, or nullptr if unknown
 */
std::unique_ptr<MotorProtocol> createProtocol(const std::string& protocol_name);

}  // namespace rs485_motor_interface

#endif  // RS485_MOTOR_INTERFACE__MOTOR_PROTOCOL_HPP_
