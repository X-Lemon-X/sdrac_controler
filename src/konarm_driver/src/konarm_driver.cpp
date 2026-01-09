#include <chrono>
#include <memory>
#include <string>

#include "ari_shared_types/status.hpp"
#include "ari_shared_types/timing.hpp"
#include "can_device/can_device.hpp"
#include "can_device/can_helper.hpp"
#include "can_device/can_messages.h"
#include "konarm_driver/konarm_driver.hpp"
#include "konarm_driver/konarm_driver_parameters.hpp"
#include "konarm_driver/konarm_joint_driver.hpp"
#include "konarm_driver/konarm_joint_driver_sim.hpp"
#include "konarm_driver_msg/msg/kon_arm_control_mode.hpp"
#include "konarm_driver_msg/srv/konarm_get_config.hpp"
#include "konarm_driver_msg/srv/konarm_set_config.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sdrac_shared_types.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

using namespace konarm_driver;


KonArmDriver::KonArmDriver() : Node("kon_arm_driver") {
  RCLCPP_INFO(this->get_logger(), "Starting KonArm Driver Node");
  on_init();
  on_activate();
}

KonArmDriver::~KonArmDriver() {
  on_deactivate();
  on_shutdown();
}

void KonArmDriver::reconfigure_callback(const Params &params) {
  RCLCPP_INFO(this->get_logger(), "Reconfigure Request:");
  RCLCPP_INFO(this->get_logger(), " frame_id: %s", params.frame_id.c_str());
  RCLCPP_INFO(this->get_logger(), "can_interface: %s", params.can_interface.c_str());
  params_ = params;
  on_deactivate();
  on_activate();
}

void KonArmDriver::main_thread_function() {
  try {

    RCLCPP_INFO(this->get_logger(), "Control thread started.");
    rclcpp::Rate rate(50); // 50 Hz control loop

    while(_active.load()) {
      auto status = control_loop();
      if(!status.ok()) {
        RCLCPP_ERROR(this->get_logger(), "Control loop error: %s", status.to_string().c_str());
        _active.store(false);
        break;
      }
      rate.sleep();
    }

  } catch(const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in control loop: %s", e.what());
    _active.store(false);
    rclcpp::shutdown();
  }
  RCLCPP_INFO(this->get_logger(), "Control thread stopping.");
}


Status KonArmDriver::on_init() {
  RCLCPP_INFO(this->get_logger(), "Init");
  param_listener_ = std::make_shared<ParamListener>(get_node_parameters_interface());
  param_listener_->setUserCallback([this](const auto &params) { reconfigure_callback(params); });
  params_ = param_listener_->get_params();

  _publisher_joint_states =
  this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::QoS(10).best_effort());
  _publisher_diagnostics =
  this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", rclcpp::QoS(10).best_effort());

  _subscription_joint_commands =
  this->create_subscription<sensor_msgs::msg::JointState>("joint_control", rclcpp::QoS(10).best_effort(),
                                                          std::bind(&KonArmDriver::subscription_joint_control_callback,
                                                                    this, std::placeholders::_1));
  _subscription_effector_commands =
  this->create_subscription<std_msgs::msg::Int8>("effector_control", rclcpp::QoS(10).best_effort(),
                                                 std::bind(&KonArmDriver::subscription_effector_control_callback,
                                                           this, std::placeholders::_1));

  _subscription_emergency_stop =
  this->create_subscription<std_msgs::msg::Bool>("emergency_stop", rclcpp::QoS(10).best_effort(),
                                                 std::bind(&KonArmDriver::subscription_emergency_stop_callback,
                                                           this, std::placeholders::_1));

  _subscription_control_mode = this->create_subscription<konarm_driver_msg::msg::KonArmControlMode>(
  "control_mode", rclcpp::QoS(10).best_effort(),
  std::bind(&KonArmDriver::subscription_control_mode_callback, this, std::placeholders::_1));

  this->create_service<konarm_driver_msg::srv::KonarmGetConfig>("konarm_get_config",
                                                                std::bind(&KonArmDriver::service_get_config_callback,
                                                                          this, std::placeholders::_1,
                                                                          std::placeholders::_2));
  this->create_service<konarm_driver_msg::srv::KonarmSetConfig>("konarm_set_config",
                                                                std::bind(&KonArmDriver::service_set_config_callback,
                                                                          this, std::placeholders::_1,
                                                                          std::placeholders::_2));

  return Status::OK();
}

Status KonArmDriver::on_activate() {
  if(_active) {
    RCLCPP_WARN(this->get_logger(), "Node is already active.");
    return Status::OK();
  }
  RCLCPP_INFO(this->get_logger(), "Activating...");


  static constexpr uint32_t base_konarm_id_mask = 0xfffffff0;
  _joint_drivers.clear();

  if(params_.use_sim_time || params_.use_sim_hardware) {
    _can_driver = nullptr;
    _joint_drivers.emplace_back(
    std::make_shared<KonArmJointDriverSimulation>(CAN_KONARM_1_STATUS_FRAME_ID & base_konarm_id_mask));
    _joint_drivers.emplace_back(
    std::make_shared<KonArmJointDriverSimulation>(CAN_KONARM_2_STATUS_FRAME_ID & base_konarm_id_mask));
    _joint_drivers.emplace_back(
    std::make_shared<KonArmJointDriverSimulation>(CAN_KONARM_3_STATUS_FRAME_ID & base_konarm_id_mask));
    _joint_drivers.emplace_back(
    std::make_shared<KonArmJointDriverSimulation>(CAN_KONARM_4_STATUS_FRAME_ID & base_konarm_id_mask));
    _joint_drivers.emplace_back(
    std::make_shared<KonArmJointDriverSimulation>(CAN_KONARM_5_STATUS_FRAME_ID & base_konarm_id_mask));
    _joint_drivers.emplace_back(
    std::make_shared<KonArmJointDriverSimulation>(CAN_KONARM_6_STATUS_FRAME_ID & base_konarm_id_mask));

  } else {
    ARI_ASIGN_TO_OR_RETURN(_can_driver, CanDriver::Make(params_.can_interface, true, 100000, 256));
    _joint_drivers.emplace_back(
    std::make_shared<KonArmJointDriver>(get_logger(), _can_driver, CAN_KONARM_1_STATUS_FRAME_ID & base_konarm_id_mask));
    _joint_drivers.emplace_back(
    std::make_shared<KonArmJointDriver>(get_logger(), _can_driver, CAN_KONARM_2_STATUS_FRAME_ID & base_konarm_id_mask));
    _joint_drivers.emplace_back(
    std::make_shared<KonArmJointDriver>(get_logger(), _can_driver, CAN_KONARM_3_STATUS_FRAME_ID & base_konarm_id_mask));
    _joint_drivers.emplace_back(
    std::make_shared<KonArmJointDriver>(get_logger(), _can_driver, CAN_KONARM_4_STATUS_FRAME_ID & base_konarm_id_mask));
    _joint_drivers.emplace_back(
    std::make_shared<KonArmJointDriver>(get_logger(), _can_driver, CAN_KONARM_5_STATUS_FRAME_ID & base_konarm_id_mask));
    _joint_drivers.emplace_back(
    std::make_shared<KonArmJointDriver>(get_logger(), _can_driver, CAN_KONARM_6_STATUS_FRAME_ID & base_konarm_id_mask));
  }
  _joint_controls.resize(_joint_drivers.size());

  _timer_errors_request = std::make_shared<ari::FrequencyTimer>(params_.error_get_hz);


  if(_control_thread.joinable()) {
    _control_thread.join();
  }
  _active         = true;
  _control_thread = std::thread(&KonArmDriver::main_thread_function, this);
  return Status::OK();
}

Status KonArmDriver::on_deactivate() {
  RCLCPP_INFO(this->get_logger(), "Deactivating...");
  if(_active) {
    _active = false;
    if(_control_thread.joinable()) {
      _control_thread.join();
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Node is already inactive.");
  }

  if(!params_.use_sim_time) {
    ARI_RETURN_ON_ERROR(CanDriver::close_can(_can_driver));
  }
  return Status::OK();
}

Status KonArmDriver::on_shutdown() {
  RCLCPP_INFO(this->get_logger(), "Shutdown");
  return Status::OK();
}

ari::Result<std::vector<double>> KonArmDriver::get_translate_from_geometry_to_joints(const std::vector<double> &robot_geometry) {
  // Example translation logic (identity in this case)
  if(robot_geometry.size() != 6) {
    RCLCPP_WARN(this->get_logger(), "Expected 6 DOF robot geometry, got %zu", robot_geometry.size());
    return Status::Invalid("Invalid robot geometry size");
  }
  std::vector<double> translated;
  translated.resize(6);
  translated[0] = robot_geometry[0]; // base rotate
  translated[1] = robot_geometry[1]; // shoulder
  translated[2] = robot_geometry[2]; // elbow
  translated[3] = robot_geometry[3]; // wrist pitch
  double out_5  = robot_geometry[4];
  double out_6  = robot_geometry[5];
  // correction fot the 4th joint
  out_5 -= robot_geometry[3];
  out_6 -= robot_geometry[3];
  // one of this wil most likely be + and the other - but that depend from the setup of the robot
  // (you shoule modify robot config rather then change this)
  out_5 = (out_6 - out_5) / 2.0; // wrist yaw
  out_6 = (out_5 + out_6) / 2.0; // wrist roll

  translated[4] = out_5;
  translated[5] = out_6;
  return ari::Result<std::vector<double>>::OK(std::move(translated));
}

ari::Result<std::vector<double>> KonArmDriver::get_translate_from_joint_to_geometry(const std::vector<double> &joint_values) {
  if(joint_values.size() != 6) {
    RCLCPP_WARN(this->get_logger(), "Expected 6 DOF robot geometry, got %zu", joint_values.size());
    return Status::Invalid("Invalid robot geometry size");
  }
  std::vector<double> translated;
  translated.resize(6);
  translated[0] = joint_values[0];                           // base rotate
  translated[1] = joint_values[1];                           // shoulder
  translated[2] = joint_values[2];                           // elbow
  translated[3] = joint_values[3];                           // wrist pitch
  double out_5  = (joint_values[4] + joint_values[5]) / 2.0; // wrist yaw
  double out_6  = (joint_values[5] - joint_values[4]) / 2.0; // wrist roll
  // correction fot the 4th joint
  // one of this wil most likely be + and the other - but that depend from the setup of the robot
  // (you shoule modify robot config rather then change this)
  out_5 += joint_values[3];
  out_6 += joint_values[3];
  translated[4] = out_5;
  translated[5] = out_6;
  return ari::Result<std::vector<double>>::OK(std::move(translated));
}

void KonArmDriver::subscription_joint_control_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  if(rclcpp::Time(msg->header.stamp) < this->get_clock()->now() - rclcpp::Duration::from_seconds(params_.msg_timeout)) {
    RCLCPP_WARN(this->get_logger(), "Received joint control message with too old timestamp");
    return;
  }

  if(msg->position.size() != _joint_controls.size() && msg->velocity.size() != _joint_controls.size() &&
     msg->effort.size() != _joint_controls.size()) {
    RCLCPP_WARN(this->get_logger(), "Received joint control message with invalid size. Expected %zu, got position %zu, velocity %zu, effort %zu",
                _joint_controls.size(), msg->position.size(), msg->velocity.size(), msg->effort.size());
    return;
  }
  auto translated_pos = get_translate_from_joint_to_geometry(msg->position).valueOrDie();
  auto translated_vel = get_translate_from_joint_to_geometry(msg->velocity).valueOrDie();
  auto translated_eff = get_translate_from_joint_to_geometry(msg->effort).valueOrDie();

  for(size_t index = 0; index < _joint_controls.size(); ++index) {
    auto &joint_control       = _joint_controls[index];
    joint_control.position_r  = translated_pos[index];
    joint_control.velocity_rs = translated_vel[index];
    joint_control.torque_nm   = translated_eff[index];
  }
}

void KonArmDriver::subscription_effector_control_callback(const std_msgs::msg::Int8::SharedPtr msg) {
  // THERE IS NO CHECK FOR OLD MESSAGE THIS IS SAFE TO DO !!! TRUST ME BRO
  int8_t command = msg->data;
  if(command < 0) {
    command = 0;
  } else if(command > 100) {
    command = 100;
  }
  for(auto &joint_driver : _joint_drivers) {
    joint_driver->set_effector_control(static_cast<uint8_t>(command));
  }
}

void KonArmDriver::subscription_emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg) {
  // THERE IS NO CHECK FOR OLD MESSAGE AS THIS IS AN EMERGENCY STOP !!!

  for(auto &joint_driver : _joint_drivers) {
    joint_driver->set_emergency_stop(msg->data);
  }
  if(msg->data) {
    RCLCPP_WARN(this->get_logger(), "KonArm EMERGENCY STOP!");
  } else {
    RCLCPP_INFO(this->get_logger(), "KonArm emergency stop deactivated.");
  }
}

void KonArmDriver::subscription_control_mode_callback(const konarm_driver_msg::msg::KonArmControlMode::SharedPtr msg) {
  MovementControlMode mode = static_cast<MovementControlMode>(msg->control_mode);
  if(mode != MovementControlMode::POSITION && mode != MovementControlMode::VELOCITY && mode != MovementControlMode::TORQUE) {
    RCLCPP_WARN(this->get_logger(), "Received KonArmControlMode message with invalid control mode %u", msg->control_mode);
    return;
  }
  if(rclcpp::Time(msg->header.stamp) < this->get_clock()->now() - rclcpp::Duration::from_seconds(params_.msg_timeout)) {
    RCLCPP_WARN(this->get_logger(), "Received KonArmControlMode message with too old timestamp");
    return;
  }


  RCLCPP_INFO(this->get_logger(), "Setting control mode to %s", to_string(mode).c_str());
  for(auto &joint_control : _joint_controls) {
    joint_control.control_mode = mode;
  }
}

void KonArmDriver::service_set_config_callback(const std::shared_ptr<konarm_driver_msg::srv::KonarmSetConfig::Request> request,
                                               std::shared_ptr<konarm_driver_msg::srv::KonarmSetConfig::Response> response) {
  //
  if(request->joint_index_to_configure >= _joint_drivers.size()) {
    response->success = false;
    RCLCPP_WARN(this->get_logger(), "Received KonarmSetConfig request with invalid joint index %u",
                request->joint_index_to_configure);
    return;
  }
  auto &joint_driver = _joint_drivers[request->joint_index_to_configure];
  auto mode          = static_cast<MovementControlMode>(request->movement_control_mode);
  if(mode != MovementControlMode::POSITION && mode != MovementControlMode::VELOCITY && mode != MovementControlMode::TORQUE) {
    RCLCPP_WARN(this->get_logger(), "Received KonarmSetConfig request with invalid movement control mode %u",
                request->movement_control_mode);
    response->success = false;
    return;
  }

  ModuleConfig config;
  config.stepper_motor_steps_per_rev              = request->stepper_motor_steps_per_rev;
  config.stepper_motor_gear_ratio                 = request->stepper_motor_gear_ratio;
  config.stepper_motor_max_velocity               = request->stepper_motor_max_velocity;
  config.stepper_motor_min_velocity               = request->stepper_motor_min_velocity;
  config.stepper_motor_reverse                    = request->stepper_motor_reverse;
  config.stepper_motor_enable_reversed            = request->stepper_motor_enable_reversed;
  config.stepper_motor_timer_prescaler            = request->stepper_motor_timer_prescaler;
  config.encoder_arm_offset                       = request->encoder_arm_offset;
  config.encoder_arm_reverse                      = request->encoder_arm_reverse;
  config.encoder_arm_dead_zone_correction_angle   = request->encoder_arm_dead_zone_correction_angle;
  config.encoder_arm_velocity_sample_amount       = request->encoder_arm_velocity_sample_amount;
  config.encoder_motor_offset                     = request->encoder_motor_offset;
  config.encoder_motor_reverse                    = request->encoder_motor_reverse;
  config.encoder_motor_dead_zone_correction_angle = request->encoder_motor_dead_zone_correction_angle;
  config.encoder_motor_velocity_sample_amount     = request->encoder_motor_velocity_sample_amount;
  config.encoder_motor_enable                     = request->encoder_motor_enable;
  config.pid_p                                    = request->pid_p;
  config.pid_i                                    = request->pid_i;
  config.pid_d                                    = request->pid_d;
  config.movement_max_velocity                    = request->movement_max_velocity;
  config.movement_limit_lower                     = request->movement_limit_lower;
  config.movement_limit_upper                     = request->movement_limit_upper;
  config.movement_control_mode                    = static_cast<uint8_t>(mode);
  config.movement_max_acceleration                = request->movement_max_acceleration;
  config.can_base_id                              = request->can_base_id;
  auto status                                     = joint_driver->set_config(config);
  response->success                               = status.ok();
  return;
}

void KonArmDriver::service_get_config_callback(const std::shared_ptr<konarm_driver_msg::srv::KonarmGetConfig::Request> request,
                                               std::shared_ptr<konarm_driver_msg::srv::KonarmGetConfig::Response> response) {
  //
  if(request->joint_index_to_get_configure >= _joint_drivers.size()) {
    response->success = false;
    RCLCPP_WARN(this->get_logger(), "Received KonarmGetConfig request with invalid joint index %u",
                request->joint_index_to_get_configure);
    return;
  }

  auto &joint_driver = _joint_drivers[request->joint_index_to_get_configure];
  auto status        = joint_driver->get_config().get_request_state();
  if(status != canc::CanStructureRequestState::RECEIVED) {
    RCLCPP_WARN(this->get_logger(), "KonarmGetConfig request for joint index %u failed, config not received.",
                request->joint_index_to_get_configure);
    response->success = false;
    return;
  }
  ModuleConfig config                                = joint_driver->get_config().get_unpacked_structure();
  response->stepper_motor_steps_per_rev              = config.stepper_motor_steps_per_rev;
  response->stepper_motor_gear_ratio                 = config.stepper_motor_gear_ratio;
  response->stepper_motor_max_velocity               = config.stepper_motor_max_velocity;
  response->stepper_motor_min_velocity               = config.stepper_motor_min_velocity;
  response->stepper_motor_reverse                    = config.stepper_motor_reverse;
  response->stepper_motor_enable_reversed            = config.stepper_motor_enable_reversed;
  response->stepper_motor_timer_prescaler            = config.stepper_motor_timer_prescaler;
  response->encoder_arm_offset                       = config.encoder_arm_offset;
  response->encoder_arm_reverse                      = config.encoder_arm_reverse;
  response->encoder_arm_dead_zone_correction_angle   = config.encoder_arm_dead_zone_correction_angle;
  response->encoder_arm_velocity_sample_amount       = config.encoder_arm_velocity_sample_amount;
  response->encoder_motor_offset                     = config.encoder_motor_offset;
  response->encoder_motor_reverse                    = config.encoder_motor_reverse;
  response->encoder_motor_dead_zone_correction_angle = config.encoder_motor_dead_zone_correction_angle;
  response->encoder_motor_velocity_sample_amount     = config.encoder_motor_velocity_sample_amount;
  response->encoder_motor_enable                     = config.encoder_motor_enable;
  response->pid_p                                    = config.pid_p;
  response->pid_i                                    = config.pid_i;
  response->pid_d                                    = config.pid_d;
  response->movement_max_velocity                    = config.movement_max_velocity;
  response->movement_limit_lower                     = config.movement_limit_lower;
  response->movement_limit_upper                     = config.movement_limit_upper;
  response->movement_control_mode                    = config.movement_control_mode;
  response->movement_max_acceleration                = config.movement_max_acceleration;
  response->can_base_id                              = config.can_base_id;
  response->success                                  = true;
  return;
}

Status KonArmDriver::control_loop() {

  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.frame_id = params_.frame_id;
  joint_state_msg.header.stamp    = this->get_clock()->now();
  // joint_state_msg.name.resize(joint_drivers_.size());
  joint_state_msg.position.resize(_joint_drivers.size());
  joint_state_msg.velocity.resize(_joint_drivers.size());
  joint_state_msg.effort.resize(_joint_drivers.size());

  diagnostic_msgs::msg::DiagnosticArray diagnostic_msg;
  diagnostic_msg.header.stamp    = this->get_clock()->now();
  diagnostic_msg.header.frame_id = params_.frame_id;
  diagnostic_msg.status.resize(_joint_drivers.size());


  for(size_t i = 0; i < _joint_drivers.size(); ++i) {
    auto &joint_driver  = _joint_drivers[i];
    auto &joint_control = _joint_controls[i];

    // Check connection timeout
    auto now = this->get_clock()->now() - joint_driver->get_module_connection_time();
    if(now.seconds() > params_.timeout_s) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "Joint with base ID %u connection timeout.", joint_driver->get_joint_base_id());
      joint_driver->request_status();
      joint_driver->reset_state();
      diagnostic_msg.status[i].name    = "joint_" + std::to_string(i + 1);
      diagnostic_msg.status[i].level   = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diagnostic_msg.status[i].message = "disconnected";
      continue;
    }

    if(joint_driver->get_control_mode() != joint_control.control_mode) {
      joint_driver->set_control_mode(joint_control.control_mode);
    }

    joint_driver->request_status();
    joint_driver->request_position();
    joint_driver->request_torque();

    if(joint_driver->get_config().get_request_state() == canc::CanStructureRequestState::NOT_REQUESTED) {
      joint_driver->request_config();
    } else if(joint_driver->get_config().get_request_state() == canc::CanStructureRequestState::RECEIVED) {
    }

    switch(joint_control.control_mode) {
    case MovementControlMode::POSITION:
      joint_driver->set_position(joint_control.position_r, joint_control.velocity_rs);
      break;
    case MovementControlMode::VELOCITY: joint_driver->set_velocity(joint_control.velocity_rs); break;
    case MovementControlMode::TORQUE: joint_driver->set_torque(joint_control.torque_nm); break;
    case MovementControlMode::NONE: break;
    default: break;
    }


    // joint_state_msg.name[i]     = "joint_" + std::to_string(i + 1);
    joint_state_msg.position[i] = joint_driver->get_position();
    joint_state_msg.velocity[i] = joint_driver->get_velocity();
    joint_state_msg.effort[i]   = joint_driver->get_torque();

    /// Diagnostics
    if(_timer_errors_request->should_trigger()) {
      joint_driver->request_errors();
      diagnostic_msg.status[i].name    = "joint_" + std::to_string(i + 1);
      diagnostic_msg.status[i].level   = joint_driver->get_status() == KonarStatus::OK ?
                                         diagnostic_msgs::msg::DiagnosticStatus::OK :
                                         diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diagnostic_msg.status[i].message = to_string(joint_driver->get_status());
      auto &dval                       = diagnostic_msg.status[i].values;
      dval.emplace_back(diagnostic_msgs::msg::KeyValue());
      dval.back().key   = "temp_engine_overheating";
      dval.back().value = std::to_string(joint_driver->get_errors().temp_engine_overheating);
      dval.emplace_back(diagnostic_msgs::msg::KeyValue());
      dval.back().key   = "temp_driver_overheating";
      dval.back().value = std::to_string(joint_driver->get_errors().temp_driver_overheating);
      dval.emplace_back(diagnostic_msgs::msg::KeyValue());
      dval.back().key   = "temp_board_overheating";
      dval.back().value = std::to_string(joint_driver->get_errors().temp_board_overheating);
      dval.emplace_back(diagnostic_msgs::msg::KeyValue());
      dval.back().key   = "temp_engine_sensor_disconnect";
      dval.back().value = std::to_string(joint_driver->get_errors().temp_engine_sensor_disconnect);
      dval.emplace_back(diagnostic_msgs::msg::KeyValue());
      dval.back().key   = "temp_driver_sensor_disconnect";
      dval.back().value = std::to_string(joint_driver->get_errors().temp_driver_sensor_disconnect);
      dval.emplace_back(diagnostic_msgs::msg::KeyValue());
      dval.back().key   = "temp_board_sensor_disconnect";
      dval.back().value = std::to_string(joint_driver->get_errors().temp_board_sensor_disconnect);
      dval.emplace_back(diagnostic_msgs::msg::KeyValue());
      dval.back().key   = "encoder_arm_disconnect";
      dval.back().value = std::to_string(joint_driver->get_errors().encoder_arm_disconnect);
      dval.emplace_back(diagnostic_msgs::msg::KeyValue());
      dval.back().key   = "encoder_motor_disconnect";
      dval.back().value = std::to_string(joint_driver->get_errors().encoder_motor_disconnect);
      dval.emplace_back(diagnostic_msgs::msg::KeyValue());
      dval.back().key   = "baord_overvoltage";
      dval.back().value = std::to_string(joint_driver->get_errors().baord_overvoltage);
      dval.emplace_back(diagnostic_msgs::msg::KeyValue());
      dval.back().key   = "baord_undervoltage";
      dval.back().value = std::to_string(joint_driver->get_errors().baord_undervoltage);
      dval.emplace_back(diagnostic_msgs::msg::KeyValue());
      dval.back().key   = "can_disconnected";
      dval.back().value = std::to_string(joint_driver->get_errors().can_disconnected);
      dval.emplace_back(diagnostic_msgs::msg::KeyValue());
      dval.back().key   = "can_error";
      dval.back().value = std::to_string(joint_driver->get_errors().can_error);
      dval.emplace_back(diagnostic_msgs::msg::KeyValue());
      dval.back().key   = "controler_motor_limit_position";
      dval.back().value = std::to_string(joint_driver->get_errors().controler_motor_limit_position);
    }
  }

  if(_timer_errors_request->should_trigger()) {
    _publisher_diagnostics->publish(diagnostic_msg);
    _timer_errors_request->reset();
  }
  _publisher_joint_states->publish(joint_state_msg);
  return Status::OK();
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KonArmDriver>());
  rclcpp::shutdown();
  return 0;
}