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

  void service_get_config_callback(const std::shared_ptr<konarm_driver_msg::srv::KonarmGetConfig::Request> request,
                                   std::shared_ptr<konarm_driver_msg::srv::KonarmGetConfig::Response> response);

  void service_set_config_callback(const std::shared_ptr<konarm_driver_msg::srv::KonarmSetConfig::Request> request,
                                   std::shared_ptr<konarm_driver_msg::srv::KonarmSetConfig::Response> response);


  /// THREADING
  std::atomic<bool> active_{ false };
  std::thread control_thread_;

  /// CAN DRIVER
  std::shared_ptr<CanDriver> can_driver_;

  /// JOINT STATES
  std::vector<KonArmJointDriver> joint_drivers_;
  std::vector<JointControl> joint_controls_;
  rclcpp::Clock clock;

  std::shared_ptr<ari::FrequencyTimer> timer_errors_request_;

  ///--------------ROS2 COMMUNICATION--------------

  /// TOPICS PUBLISHERS
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_joint_states_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_diagnostics_;

  /// TOPICS RECIEVERS
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_joint_commands_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subscription_effector_commands_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_emergency_stop_;


  /// NODE PARAMETERS
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
};

} // namespace konarm_driver