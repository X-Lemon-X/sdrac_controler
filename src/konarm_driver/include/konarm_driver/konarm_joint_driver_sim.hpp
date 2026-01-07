#pragma once


#include "ari_shared_types/status.hpp"
#include "ari_shared_types/timing.hpp"
#include "konarm_joint_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include <chrono>
#include <memory>
#include <string>


namespace konarm_driver {

class KonArmJointDriverSimulation : public KonArmJointDriverBase {
public:
  KonArmJointDriverSimulation(uint32_t joint_base_id, double p_gain = 10.0, double d_gain = 0.1)
  : KonArmJointDriverBase(), _kp(p_gain), _kd(d_gain), _clock(RCL_ROS_TIME), _joint_base_id(joint_base_id),
    _config_sender(), _prev_tim(_clock.now()) {
    set_emergency_stop(true);
  }

  virtual Status request_status() override {
    simulate();
    return Status::OK();
  }
  virtual Status request_position() override {
    simulate();
    return Status::OK();
  }
  virtual Status request_torque() override {
    simulate();
    return Status::OK();
  }
  virtual Status request_errors() override {
    simulate();
    return Status::OK();
  }
  virtual Status request_config() override {
    auto val = _config_sender.pack(_joint_base_id, _joint_control.config);
    for(const auto &msg : val) {
      (void)msg;
      _config_sender.unpack(msg);
    }
    simulate();
    return Status::OK();
  }

  virtual Status set_effector_control(uint8_t percent) override {
    _joint_control.effector_control_p = percent;
    simulate();
    return Status::OK();
  }

  virtual Status set_position(float position_r, float velocity_rs) override {
    _joint_control.position_r  = position_r;
    _joint_control.velocity_rs = velocity_rs;
    simulate();
    return Status::OK();
  }

  virtual Status set_velocity(float velocity_rs) override {
    _joint_control.velocity_rs = velocity_rs;
    simulate();
    return Status::OK();
  }

  virtual Status set_torque(float torque_nm) override {
    _joint_control.torque_nm = torque_nm;
    simulate();
    return Status::OK();
  }
  virtual Status set_control_mode(MovementControlMode mode) override {
    _joint_control.control_mode = mode;
    simulate();
    return Status::OK();
  }

  virtual Status set_config(const ModuleConfig &config) override {
    _joint_control.config = config;
    simulate();
    return Status::OK();
  }

  virtual Status set_emergency_stop(bool stop) override {
    if(stop) {
      _status                           = KonarStatus::EMERGENCY_STOP;
      _joint_control.velocity_rs        = 0;
      _joint_control.torque_nm          = 0;
      _joint_control.position_r         = 0;
      _joint_control.effector_control_p = 0;
      _robot_state.effector_control_p   = 0;
      _robot_state.position_r           = 0;
      _robot_state.velocity_rs          = 0;
      _robot_state.torque_nm            = 0;
    } else {
      _status = KonarStatus::OK;
    }
    simulate();
    return Status::OK();
  }

  virtual rclcpp::Time get_module_connection_time() const override {
    return _prev_tim;
  }

  virtual KonarStatus get_status() const override {
    return _status;
  }

  virtual MovementControlMode get_control_mode() const override {
    return _joint_control.control_mode;
  }

  virtual const ErrorData &get_errors() const override {
    static ErrorData no_errors = {};
    return no_errors;
  }

  virtual const canc::CanStructureSender<ModuleConfig> &get_config() const override {
    return _config_sender;
  }

  virtual float get_position() const override {
    return _robot_state.position_r;
  }

  virtual float get_velocity() const override {
    return _robot_state.velocity_rs;
  }

  virtual float get_torque() const override {
    return _robot_state.torque_nm;
  }

  virtual rclcpp::Time get_module_connection_time() override {
    return _clock.now();
  }

  virtual void reset_state() override {
    _joint_control = JointControl{};
    _status        = KonarStatus::EMERGENCY_STOP;
    simulate();
  }

  virtual uint32_t get_joint_base_id() const override {
    return _joint_base_id;
  }

private:
  void simulate() {
    auto current_time = _clock.now();
    double dt         = (current_time - _prev_tim).seconds();
    if(dt <= 0.001)
      return; // minimum simulation step 1ms
    _prev_tim = current_time;

    switch(_joint_control.control_mode) {
    case MovementControlMode::POSITION: {
      // Simple proportional control towards target position
      double position_error    = _joint_control.position_r - _robot_state.position_r;
      _robot_state.velocity_rs = _kp * position_error;
      _robot_state.position_r += _robot_state.velocity_rs * dt;
    } break;

    case MovementControlMode::VELOCITY: {
      _robot_state.velocity_rs = _joint_control.velocity_rs;
      _robot_state.position_r += _robot_state.velocity_rs * dt;
    } break;
    case MovementControlMode::TORQUE: {
      // Simple simulation: torque directly influences velocity
      double torque_effect = _joint_control.torque_nm * _kd; // Arbitrary factor
      _robot_state.velocity_rs += torque_effect * dt;
      _robot_state.position_r += _robot_state.velocity_rs * dt;
      _robot_state.torque_nm = _joint_control.torque_nm;
    } break;
    default: break;
    }
  }
  double _kp;
  double _kd;

  rclcpp::Clock _clock;
  uint32_t _joint_base_id;
  JointControl _joint_control;
  JointControl _robot_state;
  rclcpp::Time _prev_tim;
  canc::CanStructureSender<ModuleConfig> _config_sender;
  KonarStatus _status = KonarStatus::EMERGENCY_STOP;
};

} // namespace konarm_driver