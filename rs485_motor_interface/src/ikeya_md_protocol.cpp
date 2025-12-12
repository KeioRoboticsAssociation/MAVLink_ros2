#include "rs485_motor_interface/ikeya_md_protocol.hpp"
#include <cstring>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace rs485_motor_interface
{

IkeyaMDProtocol::IkeyaMDProtocol()
{
  // Default encoder resolution
  // Will be overridden by motor-specific configurations
}

std::vector<stm32_mavlink_msgs::msg::RS485WriteRequest>
IkeyaMDProtocol::commandToWrites(const stm32_mavlink_msgs::msg::DCMotorCommand& cmd)
{
  std::vector<stm32_mavlink_msgs::msg::RS485WriteRequest> requests;

  // Protocol v2.2 uses device_id and motor_index from mapping
  auto it = motor_mappings_.find(cmd.motor_id);
  if (it == motor_mappings_.end()) {
    // No mapping exists for this motor - skip
    return requests;  // Early return for unmapped motors
  }

  uint8_t device_id = it->second.device_id;
  uint8_t motor_index = it->second.motor_index;
  uint8_t bus_address = device_id;  // bus_address = device_id for v2.2

  uint32_t resolution = getEncoderResolution(cmd.motor_id);

  // Write operating mode (maps to control_mode register in RAM area)
  uint16_t operating_mode_addr = ikeya_md::getMotorAddress(ikeya_md::REG_OPERATING_MODE, motor_index);
  requests.push_back(createWriteRequest(
    cmd.motor_id,
    bus_address,
    operating_mode_addr,
    packUint8(static_cast<uint8_t>(cmd.control_mode))
  ));

  // Write torque enable
  uint16_t torque_enable_addr = ikeya_md::getMotorAddress(ikeya_md::REG_TORQUE_ENABLE, motor_index);
  requests.push_back(createWriteRequest(
    cmd.motor_id,
    bus_address,
    torque_enable_addr,
    packUint8(cmd.enabled ? 1 : 0)
  ));

  // Write goal value based on control mode
  switch (cmd.control_mode) {
    case 0:  // Velocity control (rad/s -> RPS)
      {
        float goal_velocity_rps = cmd.target_value / (2.0f * M_PI);  // Convert rad/s to RPS
        uint16_t goal_vel_addr = ikeya_md::getMotorAddress(ikeya_md::REG_GOAL_VELOCITY, motor_index);
        requests.push_back(createWriteRequest(
          cmd.motor_id,
          bus_address,
          goal_vel_addr,
          packFloat(goal_velocity_rps)
        ));
      }
      break;

    case 1:  // Position control (rad -> encoder counts)
      {
        int32_t goal_position = radiansTocounts(cmd.target_value, resolution);
        uint16_t goal_pos_addr = ikeya_md::getMotorAddress(ikeya_md::REG_GOAL_POSITION, motor_index);
        requests.push_back(createWriteRequest(
          cmd.motor_id,
          bus_address,
          goal_pos_addr,
          packInt32(goal_position)
        ));
      }
      break;

    case 2:  // Duty cycle control (-1.0 to 1.0)
      {
        float goal_duty = cmd.target_value;  // Already in correct range
        uint16_t goal_duty_addr = ikeya_md::getMotorAddress(ikeya_md::REG_GOAL_DUTY, motor_index);
        requests.push_back(createWriteRequest(
          cmd.motor_id,
          bus_address,
          goal_duty_addr,
          packFloat(goal_duty)
        ));
      }
      break;

    default:
      // Unknown control mode, ignore
      break;
  }

  return requests;
}

stm32_mavlink_msgs::msg::DCMotorState
IkeyaMDProtocol::parseReadResponse(
  const stm32_mavlink_msgs::msg::RS485ReadResponse& resp,
  uint8_t motor_id)
{
  stm32_mavlink_msgs::msg::DCMotorState state;
  state.motor_id = motor_id;

  uint32_t resolution = getEncoderResolution(motor_id);

  // Get motor_index from mapping
  uint8_t motor_index = 0;
  auto it = motor_mappings_.find(motor_id);
  if (it != motor_mappings_.end()) {
    motor_index = it->second.motor_index;
  } else {
    // Fallback: calculate from motor_id if no mapping exists
    motor_index = (motor_id - 30) % 3;
  }

  // Convert std::array to std::vector for easier processing
  std::vector<uint8_t> data_vec(resp.data.begin(), resp.data.begin() + resp.length);

  // Calculate base addresses for this motor
  uint16_t present_position_addr = ikeya_md::getMotorAddress(ikeya_md::REG_PRESENT_POSITION, motor_index);
  uint16_t present_velocity_addr = ikeya_md::getMotorAddress(ikeya_md::REG_PRESENT_VELOCITY, motor_index);
  uint16_t present_duty_addr = ikeya_md::getMotorAddress(ikeya_md::REG_PRESENT_DUTY, motor_index);
  uint16_t hw_error_addr = ikeya_md::getMotorAddress(ikeya_md::REG_HW_ERROR_STATUS, motor_index);
  uint16_t torque_enable_addr = ikeya_md::getMotorAddress(ikeya_md::REG_TORQUE_ENABLE, motor_index);

  // Parse based on register address
  if (resp.address == present_position_addr) {
    if (data_vec.size() >= 4) {
      int32_t position_raw = unpackInt32(data_vec);
      state.position_rad = countsToRadians(position_raw, resolution);
    }
  } else if (resp.address == present_velocity_addr) {
    if (data_vec.size() >= 4) {
      float velocity_rps = unpackFloat(data_vec);
      state.velocity_rad_s = velocity_rps * (2.0f * M_PI);  // Convert RPS to rad/s
    }
  } else if (resp.address == present_duty_addr) {
    if (data_vec.size() >= 4) {
      // float duty = unpackFloat(data_vec);
      // DCMotorState doesn't have a duty_cycle field yet
      // Could be added to the message definition if needed
    }
  } else if (resp.address == hw_error_addr) {
    if (data_vec.size() >= 1) {
      uint8_t error_flags = unpackUint8(data_vec);
      // Map error flags to status
      if (error_flags == ikeya_md::ERROR_NONE) {
        state.status = stm32_mavlink_msgs::msg::DCMotorState::STATUS_OK;
      } else if (error_flags & ikeya_md::ERROR_OVERHEAT) {
        state.status = stm32_mavlink_msgs::msg::DCMotorState::STATUS_OVERHEAT;
      } else if (error_flags & ikeya_md::ERROR_OVERCURRENT) {
        state.status = stm32_mavlink_msgs::msg::DCMotorState::STATUS_OVERCURRENT;
      } else if (error_flags & ikeya_md::ERROR_COMMUNICATION) {
        state.status = stm32_mavlink_msgs::msg::DCMotorState::STATUS_TIMEOUT;
      } else {
        state.status = stm32_mavlink_msgs::msg::DCMotorState::STATUS_ERROR;
      }
    }
  } else if (resp.address == torque_enable_addr) {
    if (data_vec.size() >= 1) {
      uint8_t enabled = unpackUint8(data_vec);
      state.enabled = (enabled != 0);
    }
  }

  return state;
}

std::vector<stm32_mavlink_msgs::msg::RS485ReadRequest>
IkeyaMDProtocol::createStatePollingRequests(uint8_t motor_id, uint8_t bus_address)
{
  std::vector<stm32_mavlink_msgs::msg::RS485ReadRequest> requests;

  // Get motor_index from mapping
  uint8_t motor_index = 0;
  auto it = motor_mappings_.find(motor_id);
  if (it != motor_mappings_.end()) {
    motor_index = it->second.motor_index;
  } else {
    // Fallback: calculate from motor_id if no mapping exists
    motor_index = (motor_id - 30) % 3;
  }

  // Read present position (4 bytes, int32)
  uint16_t pos_addr = ikeya_md::getMotorAddress(ikeya_md::REG_PRESENT_POSITION, motor_index);
  requests.push_back(createReadRequest(motor_id, bus_address, pos_addr, 4));

  // Read present velocity (4 bytes, float)
  uint16_t vel_addr = ikeya_md::getMotorAddress(ikeya_md::REG_PRESENT_VELOCITY, motor_index);
  requests.push_back(createReadRequest(motor_id, bus_address, vel_addr, 4));

  // Read present duty (4 bytes, float)
  uint16_t duty_addr = ikeya_md::getMotorAddress(ikeya_md::REG_PRESENT_DUTY, motor_index);
  requests.push_back(createReadRequest(motor_id, bus_address, duty_addr, 4));

  // Read HW error status (1 byte)
  uint16_t error_addr = ikeya_md::getMotorAddress(ikeya_md::REG_HW_ERROR_STATUS, motor_index);
  requests.push_back(createReadRequest(motor_id, bus_address, error_addr, 1));

  // Read torque enable (1 byte)
  uint16_t enable_addr = ikeya_md::getMotorAddress(ikeya_md::REG_TORQUE_ENABLE, motor_index);
  requests.push_back(createReadRequest(motor_id, bus_address, enable_addr, 1));

  return requests;
}

uint32_t IkeyaMDProtocol::getEncoderResolution(uint8_t motor_id) const
{
  auto it = encoder_resolutions_.find(motor_id);
  if (it != encoder_resolutions_.end()) {
    return it->second;
  }
  return 4096;  // Default resolution
}

void IkeyaMDProtocol::setEncoderResolution(uint8_t motor_id, uint32_t resolution)
{
  encoder_resolutions_[motor_id] = resolution;
}

void IkeyaMDProtocol::setMotorMapping(uint8_t motor_id, uint8_t device_id, uint8_t motor_index)
{
  MotorMapping mapping;
  mapping.device_id = device_id;
  mapping.motor_index = motor_index;
  motor_mappings_[motor_id] = mapping;
}

// Private helper methods

stm32_mavlink_msgs::msg::RS485WriteRequest
IkeyaMDProtocol::createWriteRequest(
  uint8_t motor_id,
  uint8_t bus_address,
  uint16_t reg_addr,
  const std::vector<uint8_t>& data) const
{
  (void)bus_address;  // bus_address is not used in the message, motor_id is sufficient
  stm32_mavlink_msgs::msg::RS485WriteRequest req;
  req.motor_id = motor_id;
  req.address = reg_addr;
  req.length = static_cast<uint8_t>(data.size());

  // Copy data to fixed-size array
  for (size_t i = 0; i < data.size() && i < 64; ++i) {
    req.data[i] = data[i];
  }

  return req;
}

stm32_mavlink_msgs::msg::RS485ReadRequest
IkeyaMDProtocol::createReadRequest(
  uint8_t motor_id,
  uint8_t bus_address,
  uint16_t reg_addr,
  uint8_t length) const
{
  (void)bus_address;  // bus_address is not used in the message, motor_id is sufficient
  stm32_mavlink_msgs::msg::RS485ReadRequest req;
  req.motor_id = motor_id;
  req.address = reg_addr;
  req.length = length;
  return req;
}

std::vector<uint8_t> IkeyaMDProtocol::packInt32(int32_t value) const
{
  std::vector<uint8_t> data(4);
  std::memcpy(data.data(), &value, 4);
  return data;
}

std::vector<uint8_t> IkeyaMDProtocol::packInt16(int16_t value) const
{
  std::vector<uint8_t> data(2);
  std::memcpy(data.data(), &value, 2);
  return data;
}

std::vector<uint8_t> IkeyaMDProtocol::packUint8(uint8_t value) const
{
  return {value};
}

std::vector<uint8_t> IkeyaMDProtocol::packFloat(float value) const
{
  std::vector<uint8_t> data(4);
  std::memcpy(data.data(), &value, 4);
  return data;
}

int32_t IkeyaMDProtocol::unpackInt32(const std::vector<uint8_t>& data, size_t offset) const
{
  if (data.size() < offset + 4) return 0;
  int32_t value;
  std::memcpy(&value, data.data() + offset, 4);
  return value;
}

int16_t IkeyaMDProtocol::unpackInt16(const std::vector<uint8_t>& data, size_t offset) const
{
  if (data.size() < offset + 2) return 0;
  int16_t value;
  std::memcpy(&value, data.data() + offset, 2);
  return value;
}

uint16_t IkeyaMDProtocol::unpackUint16(const std::vector<uint8_t>& data, size_t offset) const
{
  if (data.size() < offset + 2) return 0;
  uint16_t value;
  std::memcpy(&value, data.data() + offset, 2);
  return value;
}

uint8_t IkeyaMDProtocol::unpackUint8(const std::vector<uint8_t>& data, size_t offset) const
{
  if (data.size() < offset + 1) return 0;
  return data[offset];
}

float IkeyaMDProtocol::unpackFloat(const std::vector<uint8_t>& data, size_t offset) const
{
  if (data.size() < offset + 4) return 0.0f;
  float value;
  std::memcpy(&value, data.data() + offset, 4);
  return value;
}

}  // namespace rs485_motor_interface
