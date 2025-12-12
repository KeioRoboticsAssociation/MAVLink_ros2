#ifndef RS485_MOTOR_INTERFACE__REGISTER_MAP_HPP_
#define RS485_MOTOR_INTERFACE__REGISTER_MAP_HPP_

#include <cstdint>
#include <string>
#include <map>

namespace rs485_motor_interface
{

/**
 * @brief Register definitions for RS485 motor protocols
 *
 * This header defines register addresses and data types for common RS485 motor protocols.
 * Motor-specific implementations should extend or specialize these definitions.
 */

// RS485 Motor Register Map (Protocol v2.2 - Dynamixel-based)
// Based on RS485_PROTOCOL_v2-2.md
namespace ikeya_md
{
  // ============================================================================
  // GLOBAL PARAMETERS (Device-level, address 0x00-0x04)
  // ============================================================================
  constexpr uint16_t REG_MODEL_NUMBER_L = 0x00;    // 1 byte, uint8_t
  constexpr uint16_t REG_MODEL_NUMBER_H = 0x01;    // 1 byte, uint8_t
  constexpr uint16_t REG_FIRMWARE_VERSION = 0x02;  // 1 byte, uint8_t
  constexpr uint16_t REG_DEVICE_ID = 0x03;         // 1 byte, uint8_t (1-15)
  constexpr uint16_t REG_BAUD_RATE = 0x04;         // 1 byte, uint8_t

  // ============================================================================
  // MOTOR-SPECIFIC BASE ADDRESSES (add motor_index × 0xA0)
  // Motor 0: 0x20-0xBF, Motor 1: 0xC0-0x15F, Motor 2: 0x160-0x1FF
  // ============================================================================
  constexpr uint16_t MOTOR_OFFSET = 0xA0;          // Address offset per motor

  // EEPROM Area (0x00-0x7F) - Persistent Configuration
  constexpr uint16_t REG_OPERATING_MODE = 0x20;    // 1 byte, uint8_t (0=velocity, 1=position, 2=duty)
  constexpr uint16_t REG_MOTOR_DIRECTION = 0x21;   // 1 byte, uint8_t (0=forward, 1=reverse)
  constexpr uint16_t REG_VEL_PID_KP = 0x24;        // 4 bytes, float (0.0-100.0)
  constexpr uint16_t REG_VEL_PID_KI = 0x28;        // 4 bytes, float (0.0-10.0)
  constexpr uint16_t REG_VEL_PID_KD = 0x2C;        // 4 bytes, float (0.0-10.0)
  constexpr uint16_t REG_VEL_PID_MODE = 0x30;      // 1 byte, uint8_t (0=standard, 1=PI_D, 2=I_PD)
  constexpr uint16_t REG_POS_PID_KP = 0x34;        // 4 bytes, float (0.0-100.0)
  constexpr uint16_t REG_POS_PID_KI = 0x38;        // 4 bytes, float (0.0-10.0)
  constexpr uint16_t REG_POS_PID_KD = 0x3C;        // 4 bytes, float (0.0-10.0)
  constexpr uint16_t REG_POS_PID_MODE = 0x40;      // 1 byte, uint8_t (0=standard, 1=PI_D, 2=I_PD)
  constexpr uint16_t REG_MAX_PWM_DUTY = 0x44;      // 4 bytes, float (0.0-1.0)
  constexpr uint16_t REG_MAX_VELOCITY = 0x48;      // 4 bytes, float (0.0-1000.0 RPS)
  constexpr uint16_t REG_MAX_ACCELERATION = 0x4C;  // 4 bytes, float (0.0-10000.0 RPS²)
  constexpr uint16_t REG_MAX_POSITION = 0x50;      // 4 bytes, int32_t (encoder counts)
  constexpr uint16_t REG_MIN_POSITION = 0x54;      // 4 bytes, int32_t (encoder counts)
  constexpr uint16_t REG_ENCODER_RESOLUTION = 0x58; // 2 bytes, uint16_t (counts per revolution)

  // RAM Area (0x80-0xFF) - Volatile Runtime Parameters
  constexpr uint16_t REG_TORQUE_ENABLE = 0x80;     // 1 byte, uint8_t (0=disabled, 1=enabled)
  constexpr uint16_t REG_GOAL_VELOCITY = 0x84;     // 4 bytes, float (RPS)
  constexpr uint16_t REG_GOAL_POSITION = 0x88;     // 4 bytes, int32_t (encoder counts)
  constexpr uint16_t REG_GOAL_DUTY = 0x8C;         // 4 bytes, float (-1.0 to 1.0)
  constexpr uint16_t REG_PRESENT_VELOCITY = 0x90;  // 4 bytes, float (RPS, read-only)
  constexpr uint16_t REG_PRESENT_POSITION = 0x94;  // 4 bytes, int32_t (encoder counts, read-only)
  constexpr uint16_t REG_PRESENT_DUTY = 0x98;      // 4 bytes, float (read-only)
  constexpr uint16_t REG_HW_ERROR_STATUS = 0x9C;   // 1 byte, uint8_t (error flags, read-only)

  // ============================================================================
  // CONTROL MODES (Operating Mode Register Values)
  // ============================================================================
  enum ControlMode : uint8_t {
    MODE_VELOCITY = 0,  // Velocity control mode
    MODE_POSITION = 1,  // Position control mode (cascade PID)
    MODE_DUTY = 2       // Direct duty cycle control
  };

  // ============================================================================
  // ERROR FLAGS (HW Error Status Register Bits)
  // ============================================================================
  constexpr uint8_t ERROR_NONE = 0x00;
  constexpr uint8_t ERROR_OVERCURRENT = (1 << 0);
  constexpr uint8_t ERROR_OVERHEAT = (1 << 1);
  constexpr uint8_t ERROR_ENCODER_FAULT = (1 << 2);
  constexpr uint8_t ERROR_COMMUNICATION = (1 << 3);
  constexpr uint8_t ERROR_POSITION_LIMIT = (1 << 4);

  // ============================================================================
  // HELPER FUNCTIONS
  // ============================================================================
  /**
   * @brief Calculate register address for a specific motor
   * @param base_address Base register address (e.g., REG_GOAL_VELOCITY)
   * @param motor_index Motor index (0-2)
   * @return Actual register address for the specified motor
   */
  inline constexpr uint16_t getMotorAddress(uint16_t base_address, uint8_t motor_index) {
    return base_address + (motor_index * MOTOR_OFFSET);
  }
}

/**
 * @brief Register data type enumeration
 */
enum class RegisterType {
  UINT8,
  INT8,
  UINT16,
  INT16,
  UINT32,
  INT32,
  FLOAT
};

/**
 * @brief Register descriptor
 */
struct RegisterDescriptor {
  uint16_t address;
  RegisterType type;
  std::string name;
  bool readable;
  bool writable;

  size_t size() const {
    switch (type) {
      case RegisterType::UINT8:
      case RegisterType::INT8:
        return 1;
      case RegisterType::UINT16:
      case RegisterType::INT16:
        return 2;
      case RegisterType::UINT32:
      case RegisterType::INT32:
      case RegisterType::FLOAT:
        return 4;
      default:
        return 0;
    }
  }
};

}  // namespace rs485_motor_interface

#endif  // RS485_MOTOR_INTERFACE__REGISTER_MAP_HPP_
