#ifndef ROBOMASTER_CONTROLLER_HPP
#define ROBOMASTER_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <mutex>
#include <unordered_map>
#include "stm32_mavlink_udp/msg/robomaster_motor_command.hpp"
#include "stm32_mavlink_udp/msg/robomaster_motor_state.hpp"
#include "stm32_mavlink_udp/msg/robomaster_motor_config.hpp"
#include "stm32_mavlink_udp/srv/set_robomaster_motor_config.hpp"
#include "stm32_mavlink_udp/srv/get_robomaster_motor_config.hpp"
#include "robomaster_motor/mavlink.h"

namespace stm32_mavlink_udp {

class RobomasterController {
public:
    static constexpr uint8_t MAX_MOTORS = 8;
    static constexpr uint32_t TELEMETRY_RATE_MS = 100;  // 10Hz telemetry
    static constexpr uint32_t COMMAND_TIMEOUT_MS = 5000; // 1s command timeout

    RobomasterController(rclcpp::Node* node);
    ~RobomasterController() = default;
    
    // MAVLink message handlers for communication with STM32
    void handleMotorTelemetry(const mavlink_message_t& msg);
    void handleMotorStatus(const mavlink_message_t& msg);
    void handleParameterValue(const mavlink_message_t& msg);
    void handleCommandAck(const mavlink_message_t& msg);
    
    // Generate MAVLink messages to send to STM32
    bool getMotorControlMessage(mavlink_message_t& msg, uint8_t system_id, uint8_t component_id, uint8_t target_system);
    bool getMotorConfigMessage(mavlink_message_t& msg, uint8_t system_id, uint8_t component_id, uint8_t target_system);
    bool getParameterRequestMessage(mavlink_message_t& msg, uint8_t system_id, uint8_t component_id, uint8_t target_system);
    
    // Update method (call periodically)
    void update();

    // ROS2 callback (public for main node access)
    void motorCommandCallback(const stm32_mavlink_udp::msg::RobomasterMotorCommand::SharedPtr msg);

private:
    rclcpp::Node* node_;
    
    // ROS2 interfaces
    rclcpp::Publisher<stm32_mavlink_udp::msg::RobomasterMotorState>::SharedPtr motor_state_pub_;
    rclcpp::Service<stm32_mavlink_udp::srv::SetRobomasterMotorConfig>::SharedPtr set_config_srv_;
    rclcpp::Service<stm32_mavlink_udp::srv::GetRobomasterMotorConfig>::SharedPtr get_config_srv_;
    
    // Motor data structures
    struct MotorData {
        // Current state
        stm32_mavlink_udp::msg::RobomasterMotorState state;
        stm32_mavlink_udp::msg::RobomasterMotorConfig config;

        // Command tracking
        stm32_mavlink_udp::msg::RobomasterMotorCommand last_command;
        rclcpp::Time last_command_time;
        rclcpp::Time last_telemetry_time;
        rclcpp::Time last_update;
        bool command_pending = false;
        bool config_pending = false;
        bool online = false;

        // All motor data is now stored in the state field above
        // Removed duplicate fields to ensure consistency
    };
    
    std::unordered_map<uint8_t, MotorData> motors_;
    std::mutex motors_mutex_;
    
    // Pending requests
    struct PendingRequest {
        uint8_t motor_id;
        uint16_t param_index;
        std::string param_name;
        rclcpp::Time request_time;
    };
    std::vector<PendingRequest> pending_param_requests_;
    std::mutex pending_requests_mutex_;
    
    // Callbacks
    void setMotorConfigCallback(
        const std::shared_ptr<stm32_mavlink_udp::srv::SetRobomasterMotorConfig::Request> request,
        std::shared_ptr<stm32_mavlink_udp::srv::SetRobomasterMotorConfig::Response> response);
    void getMotorConfigCallback(
        const std::shared_ptr<stm32_mavlink_udp::srv::GetRobomasterMotorConfig::Request> request,
        std::shared_ptr<stm32_mavlink_udp::srv::GetRobomasterMotorConfig::Response> response);
    
    // Internal methods
    void publishMotorStates();
    void checkCommandTimeouts();
    void ensureMotorExists(uint8_t motor_id);
    bool isValidMotorId(uint8_t motor_id) const;
    
    // MAVLink message construction
    void buildMotorControlMessage(const stm32_mavlink_udp::msg::RobomasterMotorCommand& cmd, mavlink_message_t& msg, 
                                 uint8_t system_id, uint8_t component_id, uint8_t target_system);
    void buildParameterSetMessage(uint8_t motor_id, const std::string& param_name, float value, mavlink_message_t& msg,
                                 uint8_t system_id, uint8_t component_id, uint8_t target_system);
    
    // Parameter management
    void requestMotorParameter(uint8_t motor_id, const std::string& param_name);
    void updateMotorParameter(uint8_t motor_id, const std::string& param_name, float value);
    std::string getParameterName(uint8_t motor_id, const std::string& base_name) const;
    
    // Configuration conversion
    void configToParameters(uint8_t motor_id, const stm32_mavlink_udp::msg::RobomasterMotorConfig& config);
    void parametersToConfig(uint8_t motor_id, stm32_mavlink_udp::msg::RobomasterMotorConfig& config);
    
    // Custom MAVLink message IDs are now defined in the generated library
    // MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL = 12000
    // MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS = 12001
};

} // namespace stm32_mavlink_udp

#endif // ROBOMASTER_CONTROLLER_HPP