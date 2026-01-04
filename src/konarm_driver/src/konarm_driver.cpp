#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "ari_shared_types/status.hpp"
#include "konarm_driver/konarm_driver.hpp"

#include "can_device/can_device.hpp"
#include "can_device/can_helper.hpp"
#include "can_device/can_messages.h"


#include "konarm_driver/konarm_driver_parameters.hpp"

#include "sdrac_shared_types.hpp"

using namespace std::chrono_literals;

using namespace konarm_driver;

KonArmDriver::KonArmDriver() : Node("kon_arm_driver"), count_(0) {
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


  publisher_          = this->create_publisher<std_msgs::msg::String>("topic", 10);
  auto timer_callback = [this]() -> void {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(this->count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    this->publisher_->publish(message);
  };
  timer_ = this->create_wall_timer(500ms, timer_callback);

  return Status::OK();
}

Status KonArmDriver::on_activate() {
  if(active_) {
    RCLCPP_WARN(this->get_logger(), "Node is already active.");
    return Status::OK();
  }
  RCLCPP_INFO(this->get_logger(), "Activating...");

  ARI_ASIGN_TO_OR_RETURN(can_driver_, CanDriver::Make(params_.can_interface, true, 100000, 128));

  static constexpr uint32_t base_konarm_id_mask = 0xff0;
  std::vector<uint32_t> can_ids                 = {
    CAN_KONARM_1_STATUS_FRAME_ID & base_konarm_id_mask, CAN_KONARM_2_STATUS_FRAME_ID & base_konarm_id_mask,
    CAN_KONARM_3_STATUS_FRAME_ID & base_konarm_id_mask, CAN_KONARM_4_STATUS_FRAME_ID & base_konarm_id_mask,
    CAN_KONARM_5_STATUS_FRAME_ID & base_konarm_id_mask, CAN_KONARM_6_STATUS_FRAME_ID & base_konarm_id_mask
  };

  for(const auto &id : can_ids) {
    joint_driver_.emplace_back(get_logger(), can_driver_, id);
  }

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

Status KonArmDriver::control_loop() {

  for(auto &joint_state : joint_driver_) {
    // Check connection timeout
    auto now = clock.now();
  }


  return Status::OK();
}


KonArmJointDriver::KonArmJointDriver(rclcpp::Logger &&logger, std::shared_ptr<CanDriver> can_driver, uint32_t joint_base_id)
: logger_(std::move(logger)), config_sender(), can_driver_(can_driver), joint_base_id_(joint_base_id) {


  (void)(can_driver_->add_callback((CAN_KONARM_1_STATUS_FRAME_ID & base_command_id_mask) | joint_base_id,
                                   std::bind(&KonArmJointDriver::can_callback_status, this, std::placeholders::_1,
                                             std::placeholders::_2, std::placeholders::_3),
                                   nullptr));
  (void)(can_driver_->add_callback((CAN_KONARM_1_GET_POS_FRAME_ID & base_command_id_mask) | joint_base_id,
                                   std::bind(&KonArmJointDriver::can_callback_get_position, this,
                                             std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                                   nullptr));

  (void)(can_driver_->add_callback((CAN_KONARM_1_GET_TORQUE_FRAME_ID & base_command_id_mask) | joint_base_id,
                                   std::bind(&KonArmJointDriver::can_callback_get_torque, this,
                                             std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                                   nullptr));

  (void)(can_driver_->add_callback((CAN_KONARM_1_GET_ERRORS_FRAME_ID & base_command_id_mask) | joint_base_id,
                                   std::bind(&KonArmJointDriver::can_callback_get_errors, this,
                                             std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                                   nullptr));
  (void)(can_driver_->add_callback((CAN_KONARM_1_GET_CONFIG_FRAME_ID & base_command_id_mask) | joint_base_id,
                                   std::bind(&KonArmJointDriver::can_callback_get_config, this,
                                             std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                                   nullptr));
}

KonArmJointDriver::~KonArmJointDriver() {
  (void)can_driver_->remove_callback((CAN_KONARM_1_STATUS_FRAME_ID & 0x00f) | joint_base_id_);
  (void)can_driver_->remove_callback((CAN_KONARM_1_GET_POS_FRAME_ID & 0x00f) | joint_base_id_);
  (void)can_driver_->remove_callback((CAN_KONARM_1_GET_TORQUE_FRAME_ID & 0x00f) | joint_base_id_);
  (void)can_driver_->remove_callback((CAN_KONARM_1_GET_ERRORS_FRAME_ID & 0x00f) | joint_base_id_);
  (void)can_driver_->remove_callback((CAN_KONARM_1_GET_CONFIG_FRAME_ID & 0x00f) | joint_base_id_);
}

void KonArmJointDriver::can_callback_status(const CanDriver &driver, const CanFrame &frame, void *args) {
  (void)driver;
  (void)args;
  can_konarm_1_status_t msg;
  if(can_konarm_1_status_unpack(&msg, frame.data, frame.size) != 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to unpack status message");
    return;
  }
  status                 = static_cast<KonarStatus>(msg.status);
  module_connection_time = clock_.now();
}

void KonArmJointDriver::can_callback_get_position(const CanDriver &driver, const CanFrame &frame, void *args) {
  (void)driver;
  (void)args;
  can_konarm_1_get_pos_t msg;
  if(can_konarm_1_get_pos_unpack(&msg, frame.data, frame.size) != 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to unpack position message");
    return;
  }
  state_position_r       = msg.position;
  state_velocity_rs      = msg.velocity;
  module_connection_time = clock_.now();
}

void KonArmJointDriver::can_callback_get_torque(const CanDriver &driver, const CanFrame &frame, void *args) {
  (void)driver;
  (void)args;
  can_konarm_1_get_torque_t msg;
  if(can_konarm_1_get_torque_unpack(&msg, frame.data, frame.size) != 0) {
    RCLCPP_ERROR(get_logger(), "Failed to unpack torque message");
    return;
  }
  state_torque_nm        = msg.torque;
  module_connection_time = clock_.now();
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
  module_connection_time                = clock_.now();
}

void KonArmJointDriver::can_callback_get_config(const CanDriver &driver, const CanFrame &frame, void *args) {
  (void)driver;
  (void)args;
  canc::CanMsg can_msg;
  can_msg.id   = frame.id;
  can_msg.size = frame.size;
  std::memcpy(can_msg.data, frame.data, frame.size);
  can_msg.fdcan = false;
  if(config_sender.unpack(can_msg)) {
    config = config_sender.get_unpacked_structure();
  }
  module_connection_time = clock_.now();
}


Status KonArmJointDriver::request_status() {
  CanFrame can_msg;
  can_msg.is_remote_request = true;
  can_msg.is_extended       = CAN_KONARM_1_GET_POS_IS_EXTENDED;
  can_msg.id                = joint_base_id_ | (CAN_KONARM_1_STATUS_FRAME_ID & base_command_id_mask);
  can_msg.size              = 0;
  return can_driver_->send(can_msg);
}

Status KonArmJointDriver::request_position() {
  CanFrame can_msg;
  can_msg.is_remote_request = true;
  can_msg.is_extended       = CAN_KONARM_1_GET_POS_IS_EXTENDED;
  can_msg.id                = joint_base_id_ | (CAN_KONARM_1_GET_POS_FRAME_ID & base_command_id_mask);
  can_msg.size              = 0;
  return can_driver_->send(can_msg);
}

Status KonArmJointDriver::request_torque() {
  CanFrame can_msg;
  can_msg.is_remote_request = true;
  can_msg.is_extended       = CAN_KONARM_1_GET_TORQUE_IS_EXTENDED;
  can_msg.id                = joint_base_id_ | (CAN_KONARM_1_GET_TORQUE_FRAME_ID & base_command_id_mask);
  can_msg.size              = 0;
  return can_driver_->send(can_msg);
}

Status KonArmJointDriver::request_errors() {
  CanFrame can_msg;
  can_msg.is_remote_request = true;
  can_msg.is_extended       = CAN_KONARM_1_GET_ERRORS_IS_EXTENDED;
  can_msg.id                = joint_base_id_ | (CAN_KONARM_1_GET_ERRORS_FRAME_ID & base_command_id_mask);
  can_msg.size              = 0;
  return can_driver_->send(can_msg);
}

Status KonArmJointDriver::request_config() {
  CanFrame can_msg;
  can_msg.is_remote_request = true;
  can_msg.is_extended       = CAN_KONARM_1_GET_CONFIG_IS_EXTENDED;
  can_msg.id                = joint_base_id_ | (CAN_KONARM_1_GET_CONFIG_FRAME_ID & base_command_id_mask);
  can_msg.size              = 0;
  return can_driver_->send(can_msg);
}

Status KonArmJointDriver::set_position(float position_r, float velocity_rs) {
  can_konarm_1_set_pos_t msg;
  msg.position = position_r;
  msg.velocity = velocity_rs;

  CanFrame can_msg;
  can_msg.is_remote_request = false;
  can_msg.is_extended       = CAN_KONARM_1_SET_POS_IS_EXTENDED;
  can_msg.id                = joint_base_id_ | (CAN_KONARM_1_SET_POS_FRAME_ID & base_command_id_mask);
  can_msg.size              = CAN_KONARM_1_SET_POS_LENGTH;
  can_konarm_1_set_pos_pack(can_msg.data, &msg, can_msg.size);
  return can_driver_->send(can_msg);
}

Status KonArmJointDriver::set_velocity(float velocity_rs) {
  can_konarm_1_set_pos_t msg;
  msg.position = 0.0f; // Not used in velocity mode
  msg.velocity = velocity_rs;

  CanFrame can_msg;
  can_msg.is_remote_request = false;
  can_msg.is_extended       = CAN_KONARM_1_SET_POS_IS_EXTENDED;
  can_msg.id                = joint_base_id_ | (CAN_KONARM_1_SET_POS_FRAME_ID & base_command_id_mask);
  can_msg.size              = CAN_KONARM_1_SET_POS_LENGTH;
  can_konarm_1_set_pos_pack(can_msg.data, &msg, can_msg.size);
  return can_driver_->send(can_msg);
}

Status KonArmJointDriver::set_torque(float torque_nm) {
  can_konarm_1_set_torque_t msg;
  msg.torque = torque_nm;

  CanFrame can_msg;
  can_msg.is_remote_request = false;
  can_msg.is_extended       = CAN_KONARM_1_SET_TORQUE_IS_EXTENDED;
  can_msg.id                = joint_base_id_ | (CAN_KONARM_1_SET_TORQUE_FRAME_ID & base_command_id_mask);
  can_msg.size              = CAN_KONARM_1_SET_TORQUE_LENGTH;
  can_konarm_1_set_torque_pack(can_msg.data, &msg, can_msg.size);
  return can_driver_->send(can_msg);
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
  mode_can_msg.id   = joint_base_id_ | (CAN_KONARM_1_SET_CONTROL_MODE_FRAME_ID & base_command_id_mask);
  mode_can_msg.size = CAN_KONARM_1_SET_CONTROL_MODE_LENGTH;
  can_konarm_1_set_control_mode_pack(mode_can_msg.data, &mode_msg, mode_can_msg.size);
  return can_driver_->send(mode_can_msg);
}

Status KonArmJointDriver::set_effector_control(uint8_t percent) {
  can_konarm_1_set_effector_position_t msg;
  msg.pos_percentage = percent;

  CanFrame can_msg;
  can_msg.is_remote_request = false;
  can_msg.is_extended       = CAN_KONARM_1_SET_EFFECTOR_POSITION_IS_EXTENDED;
  can_msg.id   = joint_base_id_ | (CAN_KONARM_1_SET_EFFECTOR_POSITION_FRAME_ID & base_command_id_mask);
  can_msg.size = CAN_KONARM_1_SET_EFFECTOR_POSITION_LENGTH;
  can_konarm_1_set_effector_position_pack(can_msg.data, &msg, can_msg.size);
  return can_driver_->send(can_msg);
}

Status KonArmJointDriver::set_config(const ModuleConfig &config) {
  auto frames = config_sender.pack((CAN_KONARM_1_SEND_CONFIG_FRAME_ID & base_command_id_mask) | joint_base_id_, config);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KonArmDriver>());
  rclcpp::shutdown();
  return 0;
}