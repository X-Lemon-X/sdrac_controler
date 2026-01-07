#pragma once
// Copyright Patryk Dudziński 2025
//
// Licensed under the MIT License (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     https://opensource.org/licenses/MIT
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,


#include <chrono>
#include <memory>
#include <string>

#include "ari_shared_types/status.hpp"
#include "ari_shared_types/timing.hpp"
#include "can_device/can_device.hpp"
#include "can_device/can_helper.hpp"
#include "can_device/can_messages.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "konarm_driver/konarm_driver_parameters.hpp"
#include "konarm_driver_msg/msg/kon_arm_control_mode.hpp"
#include "konarm_driver_msg/srv/konarm_get_config.hpp"
#include "konarm_driver_msg/srv/konarm_set_config.hpp"
#include "konarm_joint_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sdrac_shared_types.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"
#include <array>
#include <atomic>
#include <thread>

using namespace std::chrono_literals;

namespace konarm_driver {

class KonArmDriver : public rclcpp::Node {
public:
  KonArmDriver();
  ~KonArmDriver();

private:
  void reconfigure_callback(const Params &params);

  Status on_init();
  Status on_activate();
  Status on_deactivate();
  Status on_shutdown();
  Status control_loop();

  void main_thread_function();

  void subscription_joint_control_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void subscription_effector_control_callback(const std_msgs::msg::Int8::SharedPtr msg);
  void subscription_emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void subscription_control_mode_callback(const konarm_driver_msg::msg::KonArmControlMode::SharedPtr msg);

  void service_get_config_callback(const std::shared_ptr<konarm_driver_msg::srv::KonarmGetConfig::Request> request,
                                   std::shared_ptr<konarm_driver_msg::srv::KonarmGetConfig::Response> response);

  void service_set_config_callback(const std::shared_ptr<konarm_driver_msg::srv::KonarmSetConfig::Request> request,
                                   std::shared_ptr<konarm_driver_msg::srv::KonarmSetConfig::Response> response);

  /// @brief Translates KonArm geometry joint values to generic joint values for 6dof arm
  /// @param robot_geometry
  /// @return Translated joint values
  ari::Result<std::vector<double>> get_translate_from_geometry_to_joints(const std::vector<double> &robot_geometry);

  /// @brief Translates generic joint 6dof arm values to KonArm geometry joint values.
  /// @param robot_geometry
  /// @return Translated joint values
  ari::Result<std::vector<double>> get_translate_from_joint_to_geometry(const std::vector<double> &robot_geometry);

  /// THREADING
  std::atomic<bool> _active{ false };
  std::thread _control_thread;

  /// CAN DRIVER
  std::shared_ptr<CanDriver> _can_driver;

  /// JOINT STATES
  std::vector<std::shared_ptr<KonArmJointDriverBase>> _joint_drivers;
  std::vector<JointControl> _joint_controls;
  rclcpp::Clock _clock;

  std::shared_ptr<ari::FrequencyTimer> _timer_errors_request;

  ///--------------ROS2 COMMUNICATION--------------

  /// TOPICS PUBLISHERS
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _publisher_joint_states;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr _publisher_diagnostics;

  /// TOPICS RECIEVERS
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _subscription_joint_commands;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr _subscription_effector_commands;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _subscription_emergency_stop;
  rclcpp::Subscription<konarm_driver_msg::msg::KonArmControlMode>::SharedPtr _subscription_control_mode;

  /// NODE PARAMETERS
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
};

} // namespace konarm_driver