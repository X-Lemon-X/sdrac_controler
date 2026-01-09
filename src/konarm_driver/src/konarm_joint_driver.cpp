#include <chrono>
#include <memory>
#include <string>

#include "ari_shared_types/status.hpp"
#include "can_device/can_device.hpp"
#include "can_device/can_helper.hpp"
#include "can_device/can_messages.h"
#include "konarm_driver/konarm_joint_driver.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sdrac_shared_types.hpp"

using namespace std::chrono_literals;

using namespace konarm_driver;

std::string konarm_driver::to_string(const KonarStatus &status) {
  switch(status) {
  case KonarStatus::OK: return "OK";
  case KonarStatus::FAULT: return "FAULT";
  case KonarStatus::OVERHEAT: return "OVERHEAT";
  case KonarStatus::EMERGENCY_STOP: return "EMERGENCY_STOP";
  default: return "UNKNOWN";
  }
};

std::string konarm_driver::to_string(const MovementControlMode &mode) {
  switch(mode) {
  case MovementControlMode::POSITION: return "POSITION";
  case MovementControlMode::VELOCITY: return "VELOCITY";
  case MovementControlMode::TORQUE: return "TORQUE";
  case MovementControlMode::NONE: return "NONE";
  default: return "UNKNOWN";
  }
};

KonArmJointDriver::KonArmJointDriver(rclcpp::Logger &&logger,
                                     rclcpp::Clock::SharedPtr clock,
                                     std::shared_ptr<CanDriver> can_driver,
                                     uint32_t joint_base_id)
: KonArmJointDriverBase(), _clock(std::move(clock)), _module_connection_time(_clock->now()), _config_sender(),
  _can_driver(can_driver), _joint_base_id(joint_base_id), _logger(std::move(logger)) {


  (void)(_can_driver->add_callback((CAN_KONARM_1_STATUS_FRAME_ID & _base_command_id_mask) | joint_base_id,
                                   std::bind(&KonArmJointDriver::can_callback_status, this, std::placeholders::_1,
                                             std::placeholders::_2, std::placeholders::_3),
                                   nullptr));
  (void)(_can_driver->add_callback((CAN_KONARM_1_GET_POS_FRAME_ID & _base_command_id_mask) | joint_base_id,
                                   std::bind(&KonArmJointDriver::can_callback_get_position, this,
                                             std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                                   nullptr));

  (void)(_can_driver->add_callback((CAN_KONARM_1_GET_TORQUE_FRAME_ID & _base_command_id_mask) | joint_base_id,
                                   std::bind(&KonArmJointDriver::can_callback_get_torque, this,
                                             std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                                   nullptr));

  (void)(_can_driver->add_callback((CAN_KONARM_1_GET_ERRORS_FRAME_ID & _base_command_id_mask) | joint_base_id,
                                   std::bind(&KonArmJointDriver::can_callback_get_errors, this,
                                             std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                                   nullptr));
  (void)(_can_driver->add_callback((CAN_KONARM_1_GET_CONFIG_FRAME_ID & _base_command_id_mask) | joint_base_id,
                                   std::bind(&KonArmJointDriver::can_callback_get_config, this,
                                             std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                                   nullptr));
}

KonArmJointDriver::~KonArmJointDriver() {
  (void)_can_driver->remove_callback((CAN_KONARM_1_STATUS_FRAME_ID & 0x00f) | _joint_base_id);
  (void)_can_driver->remove_callback((CAN_KONARM_1_GET_POS_FRAME_ID & 0x00f) | _joint_base_id);
  (void)_can_driver->remove_callback((CAN_KONARM_1_GET_TORQUE_FRAME_ID & 0x00f) | _joint_base_id);
  (void)_can_driver->remove_callback((CAN_KONARM_1_GET_ERRORS_FRAME_ID & 0x00f) | _joint_base_id);
  (void)_can_driver->remove_callback((CAN_KONARM_1_GET_CONFIG_FRAME_ID & 0x00f) | _joint_base_id);
}

void KonArmJointDriver::can_callback_status(const CanDriver &driver, const CanFrame &frame, void *args) {
  (void)driver;
  (void)args;
  can_konarm_1_status_t msg;
  if(can_konarm_1_status_unpack(&msg, frame.data, frame.size) != 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to unpack status message");
    return;
  }
  status                  = static_cast<KonarStatus>(msg.status);
  _module_connection_time = _clock->now();
}

void KonArmJointDriver::can_callback_get_position(const CanDriver &driver, const CanFrame &frame, void *args) {
  (void)driver;
  (void)args;
  can_konarm_1_get_pos_t msg;
  if(can_konarm_1_get_pos_unpack(&msg, frame.data, frame.size) != 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to unpack position message");
    return;
  }
  state_position_r        = msg.position;
  state_velocity_rs       = msg.velocity;
  _module_connection_time = _clock->now();
}

void KonArmJointDriver::can_callback_get_torque(const CanDriver &driver, const CanFrame &frame, void *args) {
  (void)driver;
  (void)args;
  can_konarm_1_get_torque_t msg;
  if(can_konarm_1_get_torque_unpack(&msg, frame.data, frame.size) != 0) {
    RCLCPP_ERROR(get_logger(), "Failed to unpack torque message");
    return;
  }
  state_torque_nm         = msg.torque;
  _module_connection_time = _clock->now();
}

void KonArmJointDriver::can_callback_get_errors(const CanDriver &driver, const CanFrame &frame, void *args) {
  (void)driver;
  (void)args;
  can_konarm_1_get_errors_t msg;
  if(can_konarm_1_get_errors_unpack(&msg, frame.data, frame.size) != 0) {
    RCLCPP_ERROR(get_logger(), "Failed to unpack errors message");
    return;
  }
  errors.temp_engine_overheating        = static_cast<bool>(msg.temp_engine_overheating);
  errors.temp_driver_overheating        = static_cast<bool>(msg.temp_driver_overheating);
  errors.temp_board_overheating         = static_cast<bool>(msg.temp_board_overheating);
  errors.temp_engine_sensor_disconnect  = static_cast<bool>(msg.temp_engine_sensor_disconnect);
  errors.temp_driver_sensor_disconnect  = static_cast<bool>(msg.temp_driver_sensor_disconnect);
  errors.temp_board_sensor_disconnect   = static_cast<bool>(msg.temp_board_sensor_disconnect);
  errors.encoder_arm_disconnect         = static_cast<bool>(msg.encoder_arm_disconnect);
  errors.encoder_motor_disconnect       = static_cast<bool>(msg.encoder_motor_disconnect);
  errors.baord_overvoltage              = static_cast<bool>(msg.board_overvoltage);
  errors.baord_undervoltage             = static_cast<bool>(msg.board_undervoltage);
  errors.can_disconnected               = static_cast<bool>(msg.can_disconnected);
  errors.can_error                      = static_cast<bool>(msg.can_error);
  errors.controler_motor_limit_position = static_cast<bool>(msg.controler_motor_limit_position);
  _module_connection_time               = _clock->now();
}

void KonArmJointDriver::can_callback_get_config(const CanDriver &driver, const CanFrame &frame, void *args) {
  (void)driver;
  (void)args;
  canc::CanMsg can_msg;
  can_msg.id   = frame.id;
  can_msg.size = frame.size;
  std::memcpy(can_msg.data, frame.data, frame.size);
  can_msg.fdcan = false;
  if(_config_sender.unpack(can_msg)) {
    config = _config_sender.get_unpacked_structure();
  }
  _module_connection_time = _clock->now();
}

Status KonArmJointDriver::request_status() {
  CanFrame can_msg;
  can_msg.is_remote_request = true;
  can_msg.is_extended       = CAN_KONARM_1_GET_POS_IS_EXTENDED;
  can_msg.id                = _joint_base_id | (CAN_KONARM_1_STATUS_FRAME_ID & _base_command_id_mask);
  can_msg.size              = 0;
  return _can_driver->send(can_msg);
}

Status KonArmJointDriver::request_position() {
  CanFrame can_msg;
  can_msg.is_remote_request = true;
  can_msg.is_extended       = CAN_KONARM_1_GET_POS_IS_EXTENDED;
  can_msg.id                = _joint_base_id | (CAN_KONARM_1_GET_POS_FRAME_ID & _base_command_id_mask);
  can_msg.size              = 0;
  return _can_driver->send(can_msg);
}

Status KonArmJointDriver::request_torque() {
  CanFrame can_msg;
  can_msg.is_remote_request = true;
  can_msg.is_extended       = CAN_KONARM_1_GET_TORQUE_IS_EXTENDED;
  can_msg.id                = _joint_base_id | (CAN_KONARM_1_GET_TORQUE_FRAME_ID & _base_command_id_mask);
  can_msg.size              = 0;
  return _can_driver->send(can_msg);
}

Status KonArmJointDriver::request_errors() {
  CanFrame can_msg;
  can_msg.is_remote_request = true;
  can_msg.is_extended       = CAN_KONARM_1_GET_ERRORS_IS_EXTENDED;
  can_msg.id                = _joint_base_id | (CAN_KONARM_1_GET_ERRORS_FRAME_ID & _base_command_id_mask);
  can_msg.size              = 0;
  return _can_driver->send(can_msg);
}

Status KonArmJointDriver::request_config() {
  CanFrame can_msg;
  can_msg.is_remote_request = true;
  can_msg.is_extended       = CAN_KONARM_1_GET_CONFIG_IS_EXTENDED;
  can_msg.id                = _joint_base_id | (CAN_KONARM_1_GET_CONFIG_FRAME_ID & _base_command_id_mask);
  can_msg.size              = 0;
  _config_sender.reset_request_state();
  return _can_driver->send(can_msg);
}

Status KonArmJointDriver::set_position(float position_r, float velocity_rs) {
  can_konarm_1_set_pos_t msg;
  msg.position = position_r;
  msg.velocity = velocity_rs;

  CanFrame can_msg;
  can_msg.is_remote_request = false;
  can_msg.is_extended       = CAN_KONARM_1_SET_POS_IS_EXTENDED;
  can_msg.id                = _joint_base_id | (CAN_KONARM_1_SET_POS_FRAME_ID & _base_command_id_mask);
  can_msg.size              = CAN_KONARM_1_SET_POS_LENGTH;
  can_konarm_1_set_pos_pack(can_msg.data, &msg, can_msg.size);
  return _can_driver->send(can_msg);
}

Status KonArmJointDriver::set_velocity(float velocity_rs) {
  can_konarm_1_set_pos_t msg;
  msg.position = 0.0f; // Not used in velocity mode
  msg.velocity = velocity_rs;

  CanFrame can_msg;
  can_msg.is_remote_request = false;
  can_msg.is_extended       = CAN_KONARM_1_SET_POS_IS_EXTENDED;
  can_msg.id                = _joint_base_id | (CAN_KONARM_1_SET_POS_FRAME_ID & _base_command_id_mask);
  can_msg.size              = CAN_KONARM_1_SET_POS_LENGTH;
  can_konarm_1_set_pos_pack(can_msg.data, &msg, can_msg.size);
  return _can_driver->send(can_msg);
}

Status KonArmJointDriver::set_torque(float torque_nm) {
  can_konarm_1_set_torque_t msg;
  msg.torque = torque_nm;

  CanFrame can_msg;
  can_msg.is_remote_request = false;
  can_msg.is_extended       = CAN_KONARM_1_SET_TORQUE_IS_EXTENDED;
  can_msg.id                = _joint_base_id | (CAN_KONARM_1_SET_TORQUE_FRAME_ID & _base_command_id_mask);
  can_msg.size              = CAN_KONARM_1_SET_TORQUE_LENGTH;
  can_konarm_1_set_torque_pack(can_msg.data, &msg, can_msg.size);
  return _can_driver->send(can_msg);
}

Status KonArmJointDriver::set_control_mode(MovementControlMode mode) {
  can_konarm_1_set_control_mode_t mode_msg;
  control_mode = mode;
  switch(mode) {
  case MovementControlMode::POSITION:
    mode_msg.control_mode = static_cast<uint8_t>(CAN_KONARM_1_SET_CONTROL_MODE_CONTROL_MODE_POSITION_CONTROL_CHOICE);
    break;
  case MovementControlMode::VELOCITY:
    mode_msg.control_mode = static_cast<uint8_t>(CAN_KONARM_1_SET_CONTROL_MODE_CONTROL_MODE_VELOCITY_CONTROL_CHOICE);
    break;
  case MovementControlMode::TORQUE:
    mode_msg.control_mode = static_cast<uint8_t>(CAN_KONARM_1_SET_CONTROL_MODE_CONTROL_MODE_TORQUE_CONTROL_CHOICE);
    break;
  default: return Status::Invalid("Invalid control mode");
  }
  CanFrame mode_can_msg;
  mode_can_msg.is_remote_request = false;
  mode_can_msg.is_extended       = CAN_KONARM_1_SET_CONTROL_MODE_IS_EXTENDED;
  mode_can_msg.id   = _joint_base_id | (CAN_KONARM_1_SET_CONTROL_MODE_FRAME_ID & _base_command_id_mask);
  mode_can_msg.size = CAN_KONARM_1_SET_CONTROL_MODE_LENGTH;
  can_konarm_1_set_control_mode_pack(mode_can_msg.data, &mode_msg, mode_can_msg.size);
  return _can_driver->send(mode_can_msg);
}

Status KonArmJointDriver::set_effector_control(uint8_t percent) {
  can_konarm_1_set_effector_position_t msg;
  msg.pos_percentage = percent;

  CanFrame can_msg;
  can_msg.is_remote_request = false;
  can_msg.is_extended       = CAN_KONARM_1_SET_EFFECTOR_POSITION_IS_EXTENDED;
  can_msg.id   = _joint_base_id | (CAN_KONARM_1_SET_EFFECTOR_POSITION_FRAME_ID & _base_command_id_mask);
  can_msg.size = CAN_KONARM_1_SET_EFFECTOR_POSITION_LENGTH;
  can_konarm_1_set_effector_position_pack(can_msg.data, &msg, can_msg.size);
  return _can_driver->send(can_msg);
}

Status KonArmJointDriver::set_config(const ModuleConfig &config) {
  auto frames =
  _config_sender.pack((CAN_KONARM_1_SEND_CONFIG_FRAME_ID & _base_command_id_mask) | _joint_base_id, config);
  for(const auto &can_msg : frames) {
    CanFrame frame;
    frame.is_remote_request = false;
    frame.is_extended       = CAN_KONARM_1_SEND_CONFIG_IS_EXTENDED;
    frame.id                = can_msg.id;
    frame.size              = can_msg.size;
    std::memcpy(frame.data, can_msg.data, can_msg.size);
    auto status = _can_driver->send(frame);
    if(!status.ok()) {
      return status;
    }
  }
  return Status::OK();
}

Status KonArmJointDriver::set_emergency_stop(bool stop) {
  can_konarm_1_status_t msg;
  msg.status = stop ? static_cast<uint8_t>(KonarStatus::EMERGENCY_STOP) : static_cast<uint8_t>(KonarStatus::OK);
  CanFrame can_msg;
  can_msg.is_remote_request = false;
  can_msg.is_extended       = CAN_KONARM_1_STATUS_IS_EXTENDED;
  can_msg.id                = _joint_base_id | (CAN_KONARM_1_STATUS_FRAME_ID & _base_command_id_mask);
  can_msg.size              = CAN_KONARM_1_STATUS_LENGTH;
  can_konarm_1_status_pack(can_msg.data, &msg, can_msg.size);
  return _can_driver->send(can_msg);
}

void KonArmJointDriver::reset_state() {
  status = KonarStatus::EMERGENCY_STOP;
  // state_position_r  = 0.0f;
  // state_velocity_rs = 0.0f;
  // state_torque_nm   = 0.0f;
  control_mode = MovementControlMode::NONE;
  errors       = ErrorData();
  _config_sender.reset_request_state();
}
