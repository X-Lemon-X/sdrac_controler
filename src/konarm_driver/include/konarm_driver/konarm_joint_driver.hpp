#pragma once
/*
Copyright (c) 2025 Patryk Dudziński

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
 * Authors: Patryk Dudziński
 */


#include <chrono>
#include <memory>
#include <string>

#include "ari_shared_types/status.hpp"
#include "can_device/can_device.hpp"
#include "can_device/can_helper.hpp"
#include "can_device/can_messages.h"
#include "rclcpp/rclcpp.hpp"
#include "sdrac_shared_types.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"
#include <array>
#include <atomic>
#include <thread>

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

std::string to_string(const KonarStatus &status);
std::string to_string(const MovementControlMode &mode);

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

  Status set_emergency_stop(bool stop);

  rclcpp::Time get_module_connection_time() const {
    return module_connection_time;
  }

  KonarStatus get_status() const {
    return status;
  }

  MovementControlMode get_control_mode() const {
    return control_mode;
  }

  const ErrorData &get_errors() const {
    return errors;
  }

  const canc::CanStructureSender<ModuleConfig> &get_config() const {
    return config_sender;
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

  void reset_state();

  uint32_t get_joint_base_id() const {
    return joint_base_id_;
  }

private:
  void can_callback_status(const CanDriver &driver, const CanFrame &frame, void *args);
  void can_callback_get_position(const CanDriver &driver, const CanFrame &frame, void *args);
  void can_callback_get_torque(const CanDriver &driver, const CanFrame &frame, void *args);
  void can_callback_get_errors(const CanDriver &driver, const CanFrame &frame, void *args);
  void can_callback_get_config(const CanDriver &driver, const CanFrame &frame, void *args);

  rclcpp::Logger &get_logger() {
    return _logger;
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
  std::shared_ptr<CanDriver> _can_driver;
  uint32_t joint_base_id_;
  rclcpp::Logger _logger;
  static constexpr uint32_t base_command_id_mask = 0x00f;
};

} // namespace konarm_driver