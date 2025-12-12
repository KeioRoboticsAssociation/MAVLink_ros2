#include "rs485_motor_interface/motor_protocol.hpp"
#include "rs485_motor_interface/ikeya_md_protocol.hpp"
#include <cmath>

namespace rs485_motor_interface
{

// Conversion utilities
int32_t MotorProtocol::radiansTocounts(double radians, uint32_t resolution) const
{
  // radians / (2*pi) * resolution = counts
  return static_cast<int32_t>(std::round(radians / (2.0 * M_PI) * resolution));
}

double MotorProtocol::countsToRadians(int32_t counts, uint32_t resolution) const
{
  // counts / resolution * 2*pi = radians
  return static_cast<double>(counts) / resolution * (2.0 * M_PI);
}

int32_t MotorProtocol::radPerSecToCounts(double rad_per_sec, uint32_t resolution) const
{
  // rad_per_sec / (2*pi) * resolution = counts/sec
  return static_cast<int32_t>(std::round(rad_per_sec / (2.0 * M_PI) * resolution));
}

double MotorProtocol::countsToRadPerSec(int32_t counts_per_sec, uint32_t resolution) const
{
  // counts_per_sec / resolution * 2*pi = rad/sec
  return static_cast<double>(counts_per_sec) / resolution * (2.0 * M_PI);
}

// Factory function
std::unique_ptr<MotorProtocol> createProtocol(const std::string& protocol_name)
{
  if (protocol_name == "ikeya_md") {
    return std::make_unique<IkeyaMDProtocol>();
  }
  // Add more protocols here as needed
  // else if (protocol_name == "other_protocol") {
  //   return std::make_unique<OtherProtocol>();
  // }

  return nullptr;  // Unknown protocol
}

}  // namespace rs485_motor_interface
