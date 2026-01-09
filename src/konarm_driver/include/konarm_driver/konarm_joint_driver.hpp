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

class KonArmJointDriverBase {
public:
  KonArmJointDriverBase() {};
  virtual ~KonArmJointDriverBase() {};
  virtual Status request_status()                                          = 0;
  virtual Status request_position()                                        = 0;
  virtual Status request_torque()                                          = 0;
  virtual Status request_errors()                                          = 0;
  virtual Status request_config()                                          = 0;
  virtual Status set_effector_control(uint8_t percent)                     = 0;
  virtual Status set_position(float position_r, float velocity_rs)         = 0;
  virtual Status set_velocity(float velocity_rs)                           = 0;
  virtual Status set_torque(float torque_nm)                               = 0;
  virtual Status set_control_mode(MovementControlMode mode)                = 0;
  virtual Status set_config(const ModuleConfig &config)                    = 0;
  virtual Status set_emergency_stop(bool stop)                             = 0;
  virtual rclcpp::Time get_module_connection_time() const                  = 0;
  virtual KonarStatus get_status() const                                   = 0;
  virtual MovementControlMode get_control_mode() const                     = 0;
  virtual const ErrorData &get_errors() const                              = 0;
  virtual const canc::CanStructureSender<ModuleConfig> &get_config() const = 0;
  virtual float get_position() const                                       = 0;
  virtual float get_velocity() const                                       = 0;
  virtual float get_torque() const                                         = 0;
  virtual rclcpp::Time get_module_connection_time()                        = 0;
  virtual void reset_state()                                               = 0;
  virtual uint32_t get_joint_base_id() const                               = 0;
};


class KonArmJointDriver : public KonArmJointDriverBase {
public:
  KonArmJointDriver(rclcpp::Logger &&logger, rclcpp::Clock::SharedPtr clock, std::shared_ptr<CanDriver> can_driver, uint32_t joint_base_id);
  ~KonArmJointDriver();

  virtual Status request_status() override;
  virtual Status request_position() override;
  virtual Status request_torque() override;
  virtual Status request_errors() override;
  virtual Status request_config() override;

  virtual Status set_effector_control(uint8_t percent) override;
  virtual Status set_position(float position_r, float velocity_rs) override;
  virtual Status set_velocity(float velocity_rs) override;
  virtual Status set_torque(float torque_nm) override;
  virtual Status set_control_mode(MovementControlMode mode) override;
  virtual Status set_config(const ModuleConfig &config) override;

  virtual Status set_emergency_stop(bool stop) override;

  virtual rclcpp::Time get_module_connection_time() const override {
    return _module_connection_time;
  }

  virtual KonarStatus get_status() const override {
    return status;
  }

  virtual MovementControlMode get_control_mode() const override {
    return control_mode;
  }

  virtual const ErrorData &get_errors() const override {
    return errors;
  }

  virtual const canc::CanStructureSender<ModuleConfig> &get_config() const override {
    return _config_sender;
  }

  virtual float get_position() const override {
    return state_position_r;
  }

  virtual float get_velocity() const override {
    return state_velocity_rs;
  }

  virtual float get_torque() const override {
    return state_torque_nm;
  }

  virtual rclcpp::Time get_module_connection_time() override {
    return _module_connection_time;
  }

  virtual void reset_state() override;

  virtual uint32_t get_joint_base_id() const override {
    return _joint_base_id;
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
  rclcpp::Clock::SharedPtr _clock;
  rclcpp::Time _module_connection_time;

  /// CAN STRUCTURE SENDERS/RECEIVERS
  canc::CanStructureSender<ModuleConfig> _config_sender;
  std::shared_ptr<CanDriver> _can_driver;
  uint32_t _joint_base_id;
  rclcpp::Logger _logger;
  static constexpr uint32_t _base_command_id_mask = 0x00f;
};

} // namespace konarm_driver