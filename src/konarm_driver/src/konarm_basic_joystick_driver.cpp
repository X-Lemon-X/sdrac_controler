#include <chrono>
#include <memory>
#include <string>

#include "ari_shared_types/status.hpp"
#include "ari_shared_types/timing.hpp"
#include "can_device/can_device.hpp"
#include "can_device/can_helper.hpp"
#include "can_device/can_messages.h"
#include "konarm_driver_msg/msg/kon_arm_control_mode.hpp"
#include "konarm_driver_msg/srv/konarm_get_config.hpp"
#include "konarm_driver_msg/srv/konarm_set_config.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sdrac_shared_types.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"

#include "konarm_driver/konarm_joint_driver.hpp"

using namespace std::chrono_literals;


class KonArmJoystickDriver : public rclcpp::Node {
public:
  KonArmJoystickDriver() : Node("kon_arm_joystick_driver") {
    _subscription_joystick = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&KonArmJoystickDriver::subscription_joystick_callback, this, std::placeholders::_1));


    _publisher_commands = this->create_publisher<sensor_msgs::msg::JointState>("joint_control", 10);
    _publisher_control_mode =
    this->create_publisher<konarm_driver_msg::msg::KonArmControlMode>("control_mode", 10);
    _publisher_emergency_stop = this->create_publisher<std_msgs::msg::Bool>("emergency_stop", 10);

    RCLCPP_INFO(this->get_logger(), "KonArm Joystick Driver Node started.");
  }

  ~KonArmJoystickDriver() {
  }

private:
  void subscription_joystick_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if(msg->axes.size() != 6 || msg->buttons.size() != _buttons_states.size()) {
      RCLCPP_WARN(this->get_logger(), "Received joystick message with invalid axes/buttons size");
      return;
    }

    sensor_msgs::msg::JointState command_msg;
    command_msg.header.frame_id = "joystick";
    command_msg.header.stamp    = this->get_clock()->now();
    command_msg.name;
    command_msg.position.resize(6);
    command_msg.effort.resize(6);
    command_msg.velocity.resize(6);

    if(check_button_pressed(2, msg)) {
      RCLCPP_INFO(this->get_logger(), "Switched to VELOCITY control mode");
      set_control_mode(konarm_driver::MovementControlMode::VELOCITY);
    } else if(check_button_pressed(3, msg)) {
      RCLCPP_INFO(this->get_logger(), "Switched to POSITION control mode");
      set_control_mode(konarm_driver::MovementControlMode::POSITION);
    } else if(check_button_pressed(4, msg)) {
      RCLCPP_INFO(this->get_logger(), "Switched to TORQUE control mode");
      set_control_mode(konarm_driver::MovementControlMode::TORQUE);
    }

    if(check_button_pressed(6, msg)) {
      emergency_stop_activated = !emergency_stop_activated;
      set_emergency_stop(emergency_stop_activated);
    }

    if(_control_mode == konarm_driver::MovementControlMode::VELOCITY) {
      std::copy(msg->axes.begin(), msg->axes.end(), command_msg.velocity.begin());
    } else if(_control_mode == konarm_driver::MovementControlMode::POSITION) {
      std::copy(msg->axes.begin(), msg->axes.end(), command_msg.position.begin());
    } else if(_control_mode == konarm_driver::MovementControlMode::TORQUE) {
      std::copy(msg->axes.begin(), msg->axes.end(), command_msg.effort.begin());
    }

    _publisher_commands->publish(command_msg);
  }

  void set_control_mode(konarm_driver::MovementControlMode mode) {
    _control_mode = mode;
    konarm_driver_msg::msg::KonArmControlMode control_mode_msg;
    control_mode_msg.header.stamp    = this->get_clock()->now();
    control_mode_msg.header.frame_id = "joystick";
    control_mode_msg.control_mode    = static_cast<uint8_t>(_control_mode);
    _publisher_control_mode->publish(control_mode_msg);
  }

  void set_emergency_stop(bool stop) {
    if(stop) {
      RCLCPP_INFO(this->get_logger(), "Activated EMERGENCY STOP");
    } else {
      RCLCPP_INFO(this->get_logger(), "Deactivated EMERGENCY STOP");
    }
    std_msgs::msg::Bool emergency_stop_msg;
    emergency_stop_msg.data = stop;
    _publisher_emergency_stop->publish(emergency_stop_msg);
  }

  bool check_button_pressed(size_t button_index, const sensor_msgs::msg::Joy::SharedPtr msg) {
    if(button_index < 0 || button_index >= _buttons_states.size()) {
      RCLCPP_WARN(this->get_logger(), "check_button_pressed: invalid button index %d", button_index);
      return false;
    }
    bool pressed                  = (msg->buttons[button_index] == 1 && _buttons_states[button_index] == 0);
    _buttons_states[button_index] = msg->buttons[button_index];
    return pressed;
  }

  konarm_driver::MovementControlMode _control_mode = konarm_driver::MovementControlMode::VELOCITY;
  std::vector<int> _buttons_states                 = std::vector<int>(10, 0);
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _subscription_joystick;
  bool emergency_stop_activated = false;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _publisher_commands;
  rclcpp::Publisher<konarm_driver_msg::msg::KonArmControlMode>::SharedPtr _publisher_control_mode;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _publisher_emergency_stop;
};


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KonArmJoystickDriver>());
  rclcpp::shutdown();
  return 0;
}