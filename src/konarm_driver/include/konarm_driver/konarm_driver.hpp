// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <string>

#include "ari_shared_types/status.hpp"
#include "can_device/can_device.hpp"
#include "can_device/can_helper.hpp"
#include "can_device/can_messages.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <array>
#include <atomic>
#include <thread>

#include "konarm_driver/konarm_driver_parameters.hpp"
#include "sdrac_shared_types.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses a fancy C++11 lambda
 * function to shorten the callback syntax, at the expense of making the
 * code somewhat more difficult to understand at first glance. */

namespace konarm_driver {

enum class KonarStatus : uint8_t {
  OK             = CAN_KONARM_1_STATUS_STATUS_OK_CHOICE,
  FAULT          = CAN_KONARM_1_STATUS_STATUS_FAULT_CHOICE,
  OVERHEAT       = CAN_KONARM_1_STATUS_STATUS_OVERHEAT_CHOICE,
  EMERGENCY_STOP = CAN_KONARM_1_STATUS_STATUS_EMERGENCY_STOP_CHOICE,
};

enum class KonarErrorState : uint8_t {
  OK    = CAN_KONARM_1_GET_ERRORS_CAN_ERROR_OK_CHOICE,
  FAULT = CAN_KONARM_1_GET_ERRORS_CAN_ERROR_FAULT_CHOICE
};

enum class MovementControlMode : uint8_t {
  POSITION = CAN_KONARM_1_SET_CONTROL_MODE_CONTROL_MODE_POSITION_CONTROL_CHOICE,
  VELOCITY = CAN_KONARM_1_SET_CONTROL_MODE_CONTROL_MODE_VELOCITY_CONTROL_CHOICE,
  TORQUE   = CAN_KONARM_1_SET_CONTROL_MODE_CONTROL_MODE_TORQUE_CONTROL_CHOICE
};


struct KonArmJointState {
  float position_r                 = 0; // radians
  float velocity_rs                = 0; // radians per second
  float torque_nm                  = 0; // Nm
  int effector_control_p           = 0; // 0-100%
  KonarStatus status               = {};
  MovementControlMode control_mode = {};
  ErrorData errors                 = {};
  ModuleConfig config              = {};
  canc::CanStructureSender<ModuleConfig> config_sender;
};


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

  void can_callback_status(const CanDriver &driver, const CanFrame &frame, void *args);
  void can_callback_get_position(const CanDriver &driver, const CanFrame &frame, void *args);
  void can_callback_get_torque(const CanDriver &driver, const CanFrame &frame, void *args);
  void can_callback_get_errors(const CanDriver &driver, const CanFrame &frame, void *args);
  void can_callback_get_config(const CanDriver &driver, const CanFrame &frame, void *args);


  /// ROS2 COMMUNICATION
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;

  /// THREADING
  std::atomic<bool> active_{ false };
  std::thread control_thread_;

  /// CAN DRIVER
  std::shared_ptr<CanDriver> can_driver_;

  /// JOINT STATES
  std::array<KonArmJointState, 6> joint_states_;

  /// NODE PARAMETERS
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
};

} // namespace konarm_driver