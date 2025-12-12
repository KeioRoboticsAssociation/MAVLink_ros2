#ifndef RS485_MOTOR_INTERFACE__IKEYA_MD_PROTOCOL_HPP_
#define RS485_MOTOR_INTERFACE__IKEYA_MD_PROTOCOL_HPP_

#include "rs485_motor_interface/motor_protocol.hpp"
#include "rs485_motor_interface/register_map.hpp"
#include <map>

namespace rs485_motor_interface
{

/**
 * @brief Protocol implementation for Ikeya MD RS485 motors
 *
 * This class implements the register-based communication protocol for Ikeya MD motors.
 * It handles conversion between ROS2 DCMotor messages and RS485 register operations.
 */
class IkeyaMDProtocol : public MotorProtocol
{
public:
  IkeyaMDProtocol();
  ~IkeyaMDProtocol() override = default;

  std::vector<stm32_mavlink_msgs::msg::RS485WriteRequest>
  commandToWrites(const stm32_mavlink_msgs::msg::DCMotorCommand& cmd) override;

  stm32_mavlink_msgs::msg::DCMotorState
  parseReadResponse(
    const stm32_mavlink_msgs::msg::RS485ReadResponse& resp,
    uint8_t motor_id) override;

  std::vector<stm32_mavlink_msgs::msg::RS485ReadRequest>
  createStatePollingRequests(uint8_t motor_id, uint8_t bus_address) override;

  std::string getProtocolName() const override {
    return "ikeya_md";
  }

  uint32_t getEncoderResolution(uint8_t motor_id) const override;
  void setEncoderResolution(uint8_t motor_id, uint32_t resolution) override;
  void setMotorMapping(uint8_t motor_id, uint8_t device_id, uint8_t motor_index) override;

private:
  /**
   * @brief Motor addressing information
   */
  struct MotorMapping {
    uint8_t device_id;
    uint8_t motor_index;
  };
  /**
   * @brief Create a write request for a register
   * @param motor_id Motor ID (30-49)
   * @param bus_address RS485 bus address
   * @param reg_addr Register address
   * @param data Data bytes to write
   * @return RS485 write request message
   */
  stm32_mavlink_msgs::msg::RS485WriteRequest createWriteRequest(
    uint8_t motor_id,
    uint8_t bus_address,
    uint16_t reg_addr,
    const std::vector<uint8_t>& data) const;

  /**
   * @brief Create a read request for a register
   * @param motor_id Motor ID (30-49)
   * @param bus_address RS485 bus address
   * @param reg_addr Register address
   * @param length Number of bytes to read
   * @return RS485 read request message
   */
  stm32_mavlink_msgs::msg::RS485ReadRequest createReadRequest(
    uint8_t motor_id,
    uint8_t bus_address,
    uint16_t reg_addr,
    uint8_t length) const;

  /**
   * @brief Pack int32_t into byte array (little-endian)
   */
  std::vector<uint8_t> packInt32(int32_t value) const;

  /**
   * @brief Pack int16_t into byte array (little-endian)
   */
  std::vector<uint8_t> packInt16(int16_t value) const;

  /**
   * @brief Pack uint8_t into byte array
   */
  std::vector<uint8_t> packUint8(uint8_t value) const;

  /**
   * @brief Pack float into byte array (little-endian)
   */
  std::vector<uint8_t> packFloat(float value) const;

  /**
   * @brief Unpack int32_t from byte array (little-endian)
   */
  int32_t unpackInt32(const std::vector<uint8_t>& data, size_t offset = 0) const;

  /**
   * @brief Unpack int16_t from byte array (little-endian)
   */
  int16_t unpackInt16(const std::vector<uint8_t>& data, size_t offset = 0) const;

  /**
   * @brief Unpack uint16_t from byte array (little-endian)
   */
  uint16_t unpackUint16(const std::vector<uint8_t>& data, size_t offset = 0) const;

  /**
   * @brief Unpack uint8_t from byte array
   */
  uint8_t unpackUint8(const std::vector<uint8_t>& data, size_t offset = 0) const;

  /**
   * @brief Unpack float from byte array (little-endian)
   */
  float unpackFloat(const std::vector<uint8_t>& data, size_t offset = 0) const;

  // Motor-specific encoder resolutions (motor_id -> resolution)
  std::map<uint8_t, uint32_t> encoder_resolutions_;

  // Motor addressing mapping (motor_id -> (device_id, motor_index))
  std::map<uint8_t, MotorMapping> motor_mappings_;
};

}  // namespace rs485_motor_interface

#endif  // RS485_MOTOR_INTERFACE__IKEYA_MD_PROTOCOL_HPP_
