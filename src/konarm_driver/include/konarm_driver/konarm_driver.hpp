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
#include <atomic>
#include <thread>

#include "konarm_driver/konarm_driver_parameters.hpp"

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

struct KonArmConfiguration {
  float stepper_motor_steps_per_rev;
  float stepper_motor_gear_ratio;
  float stepper_motor_max_velocity;
  float stepper_motor_min_velocity;
  float stepper_motor_reverse;
  float stepper_motor_enable_reversed;
  float stepper_motor_timer_prescaler;
  float encoder_arm_offset;
  float encoder_arm_reverse;
  float encoder_arm_dead_zone_correction_angle;
  float encoder_arm_velocity_sample_amount;
  float encoder_motor_offset;
  float encoder_motor_reverse;
  float encoder_motor_dead_zone_correction_angle;
  float encoder_motor_velocity_sample_amount;
  float encoder_motor_enable;
  float pid_p;
  float pid_i;
  float pid_d;
  float movement_max_velocity;
  float movement_limit_lower;
  float movement_limit_upper;
  float movement_control_mode;
  float movement_max_acceleration;
};


struct KonArmJointState {};


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

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;

  /// THREADING
  std::atomic<bool> active_{ false };
  std::thread control_thread_;

  std::shared_ptr<CanDriver> can_driver_;


  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
};

} // namespace konarm_driver