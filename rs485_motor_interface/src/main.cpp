#include "rclcpp/rclcpp.hpp"
#include "rs485_motor_interface/rs485_motor_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rs485_motor_interface::RS485MotorNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
