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
  // ARI_RETURN_ON_ERROR(can_driver_->open_can());

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

  return Status::OK();
}


void KonArmDriver::can_callback_status(const CanDriver &driver, const CanFrame &frame, void *args) {
  (void)driver;
  KonArmJointState &joint_state = *static_cast<KonArmJointState *>(args);
  can_konarm_1_status_t msg;
  if(can_konarm_1_status_unpack(&msg, frame.data, frame.size) != 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to unpack status message");
    return;
  }
  joint_state.status = static_cast<KonarStatus>(msg.status);
}

void KonArmDriver::can_callback_get_position(const CanDriver &driver, const CanFrame &frame, void *args) {
  (void)driver;
  KonArmJointState &joint_state = *static_cast<KonArmJointState *>(args);
  can_konarm_1_get_pos_t msg;
  if(can_konarm_1_get_pos_unpack(&msg, frame.data, frame.size) != 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to unpack position message");
    return;
  }
  joint_state.position_r  = msg.position;
  joint_state.velocity_rs = msg.velocity;
}

void KonArmDriver::can_callback_get_torque(const CanDriver &driver, const CanFrame &frame, void *args) {
  (void)driver;
  KonArmJointState &joint_state = *static_cast<KonArmJointState *>(args);
  can_konarm_1_get_torque_t msg;
  if(can_konarm_1_get_torque_unpack(&msg, frame.data, frame.size) != 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to unpack torque message");
    return;
  }
  joint_state.torque_nm = msg.torque;
}

void KonArmDriver::can_callback_get_errors(const CanDriver &driver, const CanFrame &frame, void *args) {
  (void)driver;
  KonArmJointState &joint_state = *static_cast<KonArmJointState *>(args);
  can_konarm_1_get_errors_t msg;
  if(can_konarm_1_get_errors_unpack(&msg, frame.data, frame.size) != 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to unpack errors message");
    return;
  }
  joint_state.errors.temp_engine_overheating        = static_cast<bool>(msg.temp_engine_overheating);
  joint_state.errors.temp_driver_overheating        = static_cast<bool>(msg.temp_driver_overheating);
  joint_state.errors.temp_board_overheating         = static_cast<bool>(msg.temp_board_overheating);
  joint_state.errors.temp_engine_sensor_disconnect  = static_cast<bool>(msg.temp_engine_sensor_disconnect);
  joint_state.errors.temp_driver_sensor_disconnect  = static_cast<bool>(msg.temp_driver_sensor_disconnect);
  joint_state.errors.temp_board_sensor_disconnect   = static_cast<bool>(msg.temp_board_sensor_disconnect);
  joint_state.errors.encoder_arm_disconnect         = static_cast<bool>(msg.encoder_arm_disconnect);
  joint_state.errors.encoder_motor_disconnect       = static_cast<bool>(msg.encoder_motor_disconnect);
  joint_state.errors.baord_overvoltage              = static_cast<bool>(msg.board_overvoltage);
  joint_state.errors.baord_undervoltage             = static_cast<bool>(msg.board_undervoltage);
  joint_state.errors.can_disconnected               = static_cast<bool>(msg.can_disconnected);
  joint_state.errors.can_error                      = static_cast<bool>(msg.can_error);
  joint_state.errors.controler_motor_limit_position = static_cast<bool>(msg.controler_motor_limit_position);

  // joint_state.errors.motor_error                  = static_cast<KonarErrorState>(msg.motor_error);
}

void KonArmDriver::can_callback_get_config(const CanDriver &driver, const CanFrame &frame, void *args) {
  (void)driver;
  KonArmJointState &joint_state = *static_cast<KonArmJointState *>(args);

  canc::CanMsg can_msg;
  can_msg.id   = frame.id;
  can_msg.size = frame.size;
  std::memcpy(can_msg.data, frame.data, frame.size);
  can_msg.fdcan = false;
  if(joint_state.config_sender.unpack(can_msg)) {
    joint_state.config = joint_state.config_sender.get_unpacked_structure();
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KonArmDriver>());
  rclcpp::shutdown();
  return 0;
}