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

    while(active_) {
      auto status = control_loop();
      if(!status.ok()) {
        RCLCPP_ERROR(this->get_logger(), "Control loop error: %s", status.to_string().c_str());
        active_ = false;
        break;
      }
      rate.sleep();
    }

  } catch(const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in control loop: %s", e.what());
  }
  RCLCPP_INFO(this->get_logger(), "Control thread stopping.");
}

Status KonArmDriver::on_init() {
  RCLCPP_INFO(this->get_logger(), "Init");
  param_listener_ = std::make_shared<ParamListener>(get_node_parameters_interface());
  param_listener_->setUserCallback([this](const auto &params) { reconfigure_callback(params); });
  params_ = param_listener_->get_params();

  publisher_joint_states_ =
  this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::QoS(10).best_effort());
  publisher_diagnostics_ =
  this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", rclcpp::QoS(10).best_effort());

  subscription_joint_commands_ =
  this->create_subscription<sensor_msgs::msg::JointState>("joint_control", rclcpp::QoS(10).best_effort(),
                                                          std::bind(&KonArmDriver::subscription_joint_control_callback,
                                                                    this, std::placeholders::_1));
  subscription_effector_commands_ =
  this->create_subscription<std_msgs::msg::Int8>("effector_control", rclcpp::QoS(10).best_effort(),
                                                 std::bind(&KonArmDriver::subscription_effector_control_callback,
                                                           this, std::placeholders::_1));

  subscription_emergency_stop_ =
  this->create_subscription<std_msgs::msg::Bool>("emergency_stop", rclcpp::QoS(10).best_effort(),
                                                 std::bind(&KonArmDriver::subscription_emergency_stop_callback,
                                                           this, std::placeholders::_1));

  this->create_service<konarm_driver_msg::srv::KonarmGetConfig>("konarm_get_config",
                                                                std::bind(&KonArmDriver::service_get_config_callback,
                                                                          this, std::placeholders::_1,
                                                                          std::placeholders::_2));
  this->create_service<konarm_driver_msg::srv::KonarmSetConfig>("konarm_set_config",
                                                                std::bind(&KonArmDriver::service_set_config_callback,
                                                                          this, std::placeholders::_1,
                                                                          std::placeholders::_2));


  // publisher_          = this->create_publisher<std_msgs::msg::String>("topic", 10);
  // auto timer_callback = [this]() -> void {
  //   auto message = std_msgs::msg::String();
  //   message.data = "Hello, world! " + std::to_string(this->count_++);
  //   RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  //   this->publisher_->publish(message);
  // };
  // timer_ = this->create_wall_timer(500ms, timer_callback);

  return Status::OK();
}

Status KonArmDriver::on_activate() {
  if(active_) {
    RCLCPP_WARN(this->get_logger(), "Node is already active.");
    return Status::OK();
  }
  RCLCPP_INFO(this->get_logger(), "Activating...");

  ARI_ASIGN_TO_OR_RETURN(can_driver_, CanDriver::Make(params_.can_interface, true, 100000, 256));

  static constexpr uint32_t base_konarm_id_mask = 0xff0;
  joint_drivers_.emplace_back(get_logger(), can_driver_, CAN_KONARM_1_STATUS_FRAME_ID & base_konarm_id_mask);
  joint_drivers_.emplace_back(get_logger(), can_driver_, CAN_KONARM_2_STATUS_FRAME_ID & base_konarm_id_mask);
  joint_drivers_.emplace_back(get_logger(), can_driver_, CAN_KONARM_3_STATUS_FRAME_ID & base_konarm_id_mask);
  joint_drivers_.emplace_back(get_logger(), can_driver_, CAN_KONARM_4_STATUS_FRAME_ID & base_konarm_id_mask);
  joint_drivers_.emplace_back(get_logger(), can_driver_, CAN_KONARM_5_STATUS_FRAME_ID & base_konarm_id_mask);
  joint_drivers_.emplace_back(get_logger(), can_driver_, CAN_KONARM_6_STATUS_FRAME_ID & base_konarm_id_mask);

  joint_controls_.resize(joint_drivers_.size());

  timer_errors_request_ = std::make_shared<ari::FrequencyTimer>(params_.error_get_hz);


  if(control_thread_.joinable()) {
    control_thread_.join();
  }
  active_         = true;
  control_thread_ = std::thread(&KonArmDriver::main_thread_function, this);
  return Status::OK();
}

Status KonArmDriver::on_deactivate() {
  RCLCPP_INFO(this->get_logger(), "Deactivating...");
  if(active_) {
    active_ = false;
    if(control_thread_.joinable()) {
      control_thread_.join();
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Node is already inactive.");
  }

  CanDriver::close_can(can_driver_);
  return Status::OK();
}

Status KonArmDriver::on_shutdown() {
  RCLCPP_INFO(this->get_logger(), "Shutdown");
  return Status::OK();
}

void KonArmDriver::subscription_joint_control_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  for(size_t index = 0; index < joint_controls_.size(); ++index) {
    auto &joint_control = joint_controls_[index];
    // Find index of joint in message
    if(index < msg->position.size()) {
      joint_control.position_r = msg->position[index];
    }
    if(index < msg->velocity.size()) {
      joint_control.velocity_rs = msg->velocity[index];
    }
    if(index < msg->effort.size()) {
      joint_control.torque_nm = msg->effort[index];
    }

    // Determine control mode based on which fields are set
    if(index < msg->position.size()) {
      joint_control.control_mode = MovementControlMode::POSITION;
    } else if(index < msg->velocity.size()) {
      joint_control.control_mode = MovementControlMode::VELOCITY;
    } else if(index < msg->effort.size()) {
      joint_control.control_mode = MovementControlMode::TORQUE;
    } else {
      joint_control.control_mode = MovementControlMode::NONE;
    }
  }
}

void KonArmDriver::subscription_effector_control_callback(const std_msgs::msg::Int8::SharedPtr msg) {
  int8_t command = msg->data;
  if(command < 0) {
    command = 0;
  } else if(command > 100) {
    command = 100;
  }
  for(auto &joint_driver : joint_drivers_) {
    joint_driver.set_effector_control(static_cast<uint8_t>(command));
  }
}

void KonArmDriver::subscription_emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg) {
  for(auto &joint_driver : joint_drivers_) {
    joint_driver.set_emergency_stop(msg->data);
  }
  if(msg->data) {
    RCLCPP_WARN(this->get_logger(), "KonArm EMERGENCY STOP!");
  } else {
    RCLCPP_INFO(this->get_logger(), "KonArm emergency stop deactivated.");
  }
}

void KonArmDriver::service_set_config_callback(const std::shared_ptr<konarm_driver_msg::srv::KonarmSetConfig::Request> request,
                                               std::shared_ptr<konarm_driver_msg::srv::KonarmSetConfig::Response> response) {
  //
  if(request->joint_index_to_configure >= joint_drivers_.size()) {
    response->success = false;
    RCLCPP_WARN(this->get_logger(), "Received KonarmSetConfig request with invalid joint index %u",
                request->joint_index_to_configure);
    return;
  }
  auto &joint_driver = joint_drivers_[request->joint_index_to_configure];
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
  auto status                                     = joint_driver.set_config(config);
  response->success                               = status.ok();
  return;
}

void KonArmDriver::service_get_config_callback(const std::shared_ptr<konarm_driver_msg::srv::KonarmGetConfig::Request> request,
                                               std::shared_ptr<konarm_driver_msg::srv::KonarmGetConfig::Response> response) {
  //
  if(request->joint_index_to_get_configure >= joint_drivers_.size()) {
    response->success = false;
    RCLCPP_WARN(this->get_logger(), "Received KonarmGetConfig request with invalid joint index %u",
                request->joint_index_to_get_configure);
    return;
  }

  auto &joint_driver = joint_drivers_[request->joint_index_to_get_configure];
  auto status        = joint_driver.get_config().get_request_state();
  if(status != canc::CanStructureRequestState::RECEIVED) {
    RCLCPP_WARN(this->get_logger(), "KonarmGetConfig request for joint index %u failed, config not received.",
                request->joint_index_to_get_configure);
    response->success = false;
    return;
  }
  ModuleConfig config                                = joint_driver.get_config().get_unpacked_structure();
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
  joint_state_msg.header.stamp    = clock.now();
  // joint_state_msg.name.resize(joint_drivers_.size());
  joint_state_msg.position.resize(joint_drivers_.size());
  joint_state_msg.velocity.resize(joint_drivers_.size());
  joint_state_msg.effort.resize(joint_drivers_.size());

  diagnostic_msgs::msg::DiagnosticArray diagnostic_msg;
  diagnostic_msg.header.stamp    = clock.now();
  diagnostic_msg.header.frame_id = params_.frame_id;
  diagnostic_msg.status.resize(joint_drivers_.size());


  for(size_t i = 0; i < joint_drivers_.size(); ++i) {
    auto &joint_driver  = joint_drivers_[i];
    auto &joint_control = joint_controls_[i];

    // Check connection timeout
    auto now = clock.now() - joint_driver.get_module_connection_time();
    if(now.seconds() > params_.timeout_s) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 5000, "Joint with base ID %u connection timeout.",
                           joint_driver.get_joint_base_id());
      joint_driver.request_status();
      joint_driver.reset_state();
      diagnostic_msg.status[i].name    = "joint_" + std::to_string(i + 1);
      diagnostic_msg.status[i].level   = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diagnostic_msg.status[i].message = "disconnected";
      continue;
    }

    if(joint_driver.get_control_mode() != joint_control.control_mode) {
      joint_driver.set_control_mode(joint_control.control_mode);
    }

    joint_driver.request_status();
    joint_driver.request_position();
    joint_driver.request_torque();

    if(joint_driver.get_config().get_request_state() == canc::CanStructureRequestState::NOT_REQUESTED) {
      joint_driver.request_config();
    } else if(joint_driver.get_config().get_request_state() == canc::CanStructureRequestState::RECEIVED) {
    }

    switch(joint_control.control_mode) {
    case MovementControlMode::POSITION:
      joint_driver.set_position(joint_control.position_r, joint_control.velocity_rs);
      break;
    case MovementControlMode::VELOCITY: joint_driver.set_velocity(joint_control.velocity_rs); break;
    case MovementControlMode::TORQUE: joint_driver.set_torque(joint_control.torque_nm); break;
    case MovementControlMode::NONE: break;
    default: break;
    }


    // joint_state_msg.name[i]     = "joint_" + std::to_string(i + 1);
    joint_state_msg.position[i] = joint_driver.get_position();
    joint_state_msg.velocity[i] = joint_driver.get_velocity();
    joint_state_msg.effort[i]   = joint_driver.get_torque();

    /// Diagnostics
    if(timer_errors_request_->should_trigger()) {
      joint_driver.request_errors();
      diagnostic_msg.status[i].name    = "joint_" + std::to_string(i + 1);
      diagnostic_msg.status[i].level   = joint_driver.get_status() == KonarStatus::OK ?
                                         diagnostic_msgs::msg::DiagnosticStatus::OK :
                                         diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diagnostic_msg.status[i].message = to_string(joint_driver.get_status());
      auto &dval                       = diagnostic_msg.status[i].values;
      dval.emplace_back(diagnostic_msgs::msg::KeyValue());
      dval.back().key   = "temp_engine_overheating";
      dval.back().value = std::to_string(joint_driver.get_errors().temp_engine_overheating);
      dval.emplace_back(diagnostic_msgs::msg::KeyValue());
      dval.back().key   = "temp_driver_overheating";
      dval.back().value = std::to_string(joint_driver.get_errors().temp_driver_overheating);
      dval.emplace_back(diagnostic_msgs::msg::KeyValue());
      dval.back().key   = "temp_board_overheating";
      dval.back().value = std::to_string(joint_driver.get_errors().temp_board_overheating);
      dval.emplace_back(diagnostic_msgs::msg::KeyValue());
      dval.back().key   = "temp_engine_sensor_disconnect";
      dval.back().value = std::to_string(joint_driver.get_errors().temp_engine_sensor_disconnect);
      dval.emplace_back(diagnostic_msgs::msg::KeyValue());
      dval.back().key   = "temp_driver_sensor_disconnect";
      dval.back().value = std::to_string(joint_driver.get_errors().temp_driver_sensor_disconnect);
      dval.emplace_back(diagnostic_msgs::msg::KeyValue());
      dval.back().key   = "temp_board_sensor_disconnect";
      dval.back().value = std::to_string(joint_driver.get_errors().temp_board_sensor_disconnect);
      dval.emplace_back(diagnostic_msgs::msg::KeyValue());
      dval.back().key   = "encoder_arm_disconnect";
      dval.back().value = std::to_string(joint_driver.get_errors().encoder_arm_disconnect);
      dval.emplace_back(diagnostic_msgs::msg::KeyValue());
      dval.back().key   = "encoder_motor_disconnect";
      dval.back().value = std::to_string(joint_driver.get_errors().encoder_motor_disconnect);
      dval.emplace_back(diagnostic_msgs::msg::KeyValue());
      dval.back().key   = "baord_overvoltage";
      dval.back().value = std::to_string(joint_driver.get_errors().baord_overvoltage);
      dval.emplace_back(diagnostic_msgs::msg::KeyValue());
      dval.back().key   = "baord_undervoltage";
      dval.back().value = std::to_string(joint_driver.get_errors().baord_undervoltage);
      dval.emplace_back(diagnostic_msgs::msg::KeyValue());
      dval.back().key   = "can_disconnected";
      dval.back().value = std::to_string(joint_driver.get_errors().can_disconnected);
      dval.emplace_back(diagnostic_msgs::msg::KeyValue());
      dval.back().key   = "can_error";
      dval.back().value = std::to_string(joint_driver.get_errors().can_error);
      dval.emplace_back(diagnostic_msgs::msg::KeyValue());
      dval.back().key   = "controler_motor_limit_position";
      dval.back().value = std::to_string(joint_driver.get_errors().controler_motor_limit_position);
    }
  }

  if(timer_errors_request_->should_trigger()) {
    publisher_diagnostics_->publish(diagnostic_msg);
    timer_errors_request_->reset();
  }


  publisher_joint_states_->publish(joint_state_msg);

  return Status::OK();
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KonArmDriver>());
  rclcpp::shutdown();
  return 0;
}