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
  TORQUE   = CAN_KONARM_1_SET_CONTROL_MODE_CONTROL_MODE_TORQUE_CONTROL_CHOICE,
  NONE
};

struct JointControl {
  int effector_control_p           = 0; // 0-100%
  float position_r                 = 0; // radians
  float velocity_rs                = 0; // radians per second
  float torque_nm                  = 0; // Nm
  ModuleConfig config              = {};
  MovementControlMode control_mode = MovementControlMode::VELOCITY;
};


class KonArmJointDriver {
public:
  KonArmJointDriver(rclcpp::Logger &&logger, std::shared_ptr<CanDriver> can_driver, uint32_t joint_base_id);
  ~KonArmJointDriver();

  Status request_status();
  Status request_position();
  Status request_torque();
  Status request_errors();
  Status request_config();

  Status set_effector_control(uint8_t percent);
  Status set_position(float position_r, float velocity_rs);
  Status set_velocity(float velocity_rs);
  Status set_torque(float torque_nm);
  Status set_control_mode(MovementControlMode mode);
  Status set_config(const ModuleConfig &config);

  rclcpp::Time get_module_connection_time() const {
    return module_connection_time;
  }

  KonarStatus get_status() const {
    return status;
  }

  MovementControlMode get_control_mode() const {
    return control_mode;
  }

  ErrorData get_errors() const {
    return errors;
  }

  ModuleConfig get_config() const {
    return config;
  }

  float get_position() const {
    return state_position_r;
  }

  float get_velocity() const {
    return state_velocity_rs;
  }

  float get_torque() const {
    return state_torque_nm;
  }

  rclcpp::Time get_module_connection_time() {
    return module_connection_time;
  }

private:
  void can_callback_status(const CanDriver &driver, const CanFrame &frame, void *args);
  void can_callback_get_position(const CanDriver &driver, const CanFrame &frame, void *args);
  void can_callback_get_torque(const CanDriver &driver, const CanFrame &frame, void *args);
  void can_callback_get_errors(const CanDriver &driver, const CanFrame &frame, void *args);
  void can_callback_get_config(const CanDriver &driver, const CanFrame &frame, void *args);

  rclcpp::Logger &get_logger() {
    return logger_;
  };


  /// JOINT STATE VARIABLES
  float state_position_r  = 0; // radians
  float state_velocity_rs = 0; // radians per second
  float state_torque_nm   = 0; // Nm


  KonarStatus status               = KonarStatus::EMERGENCY_STOP;
  MovementControlMode control_mode = MovementControlMode::NONE;
  ErrorData errors                 = {};
  ModuleConfig config              = {};

  rclcpp::Time module_connection_time;
  rclcpp::Clock clock_;

  /// CAN STRUCTURE SENDERS/RECEIVERS
  canc::CanStructureSender<ModuleConfig> config_sender;
  std::shared_ptr<CanDriver> can_driver_;
  uint32_t joint_base_id_;
  static constexpr uint32_t base_command_id_mask = 0x00f;
  rclcpp::Logger logger_;
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
  std::vector<KonArmJointDriver> joint_driver_;
  rclcpp::Clock clock;

  /// NODE PARAMETERS
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
};

} // namespace konarm_driver