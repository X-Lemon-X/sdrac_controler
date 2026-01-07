# konarm_driver_msg

ROS 2 message and service definitions for the KonARM robotic arm driver system.

## Overview

This package provides the interface definitions (messages and services) used for communication with KonARM robotic arm joints. It defines control modes, configuration parameters, and service interfaces for runtime configuration of individual joints.

**Version:** 0.1.0
**Maintainer:** Patryk Dudziński (dodpat02@gmail.com)
**License:** MIT

## Package Contents

### Messages

#### KonArmControlMode.msg

Defines the control mode for a joint's movement.

```msg
std_msgs/Header header
uint8 POSITION=0
uint8 VELOCITY=1
uint8 TORQUE=2
uint8 control_mode
```

**Constants:**
- `POSITION` (0): Position control mode - joint moves to a target position
- `VELOCITY` (1): Velocity control mode - joint maintains a target velocity
- `TORQUE` (2): Torque control mode - joint applies a target torque

**Fields:**
- `header`: Standard ROS message header with timestamp and frame information
- `control_mode`: Current or desired control mode (0-2)

### Services

#### KonarmGetConfig.srv

Retrieves the current configuration parameters for a specific joint.

**Request:**
```msg
uint8 joint_index_to_get_configure
```

**Response:**
```msg
# Stepper Motor Configuration
float32 stepper_motor_steps_per_rev
float32 stepper_motor_gear_ratio
float32 stepper_motor_max_velocity
float32 stepper_motor_min_velocity
bool stepper_motor_reverse
bool stepper_motor_enable_reversed
uint32 stepper_motor_timer_prescaler

# Encoder Arm Configuration
float32 encoder_arm_offset
bool encoder_arm_reverse
float32 encoder_arm_dead_zone_correction_angle
uint16 encoder_arm_velocity_sample_amount

# Encoder Motor Configuration
float32 encoder_motor_offset
bool encoder_motor_reverse
float32 encoder_motor_dead_zone_correction_angle
uint16 encoder_motor_velocity_sample_amount
bool encoder_motor_enable

# PID Configuration
float32 pid_p
float32 pid_i
float32 pid_d

# Movement Configuration
float32 movement_max_velocity
float32 movement_limit_lower
float32 movement_limit_upper
uint8 movement_control_mode
float32 movement_max_acceleration

# CAN Configuration
uint16 can_base_id

bool success
```

#### KonarmSetConfig.srv

Updates the configuration parameters for a specific joint.

**Request:**
```msg
uint8 joint_index_to_configure

# Stepper Motor Configuration
float32 stepper_motor_steps_per_rev
float32 stepper_motor_gear_ratio
float32 stepper_motor_max_velocity
float32 stepper_motor_min_velocity
bool stepper_motor_reverse
bool stepper_motor_enable_reversed
uint32 stepper_motor_timer_prescaler

# Encoder Arm Configuration
float32 encoder_arm_offset
bool encoder_arm_reverse
float32 encoder_arm_dead_zone_correction_angle
uint16 encoder_arm_velocity_sample_amount

# Encoder Motor Configuration
float32 encoder_motor_offset
bool encoder_motor_reverse
float32 encoder_motor_dead_zone_correction_angle
uint16 encoder_motor_velocity_sample_amount
bool encoder_motor_enable

# PID Configuration
float32 pid_p
float32 pid_i
float32 pid_d

# Movement Configuration
float32 movement_max_velocity
float32 movement_limit_lower
float32 movement_limit_upper
uint8 movement_control_mode
float32 movement_max_acceleration

# CAN Configuration
uint16 can_base_id
```

**Response:**
```msg
bool success
```

## Configuration Parameters

### Stepper Motor Parameters
- **steps_per_rev**: Number of steps per motor revolution
- **gear_ratio**: Gear reduction ratio between motor and joint
- **max_velocity / min_velocity**: Velocity limits (rad/s)
- **reverse**: Invert motor direction
- **enable_reversed**: Invert enable signal logic
- **timer_prescaler**: Hardware timer prescaler value

### Encoder Parameters (Arm & Motor)
- **offset**: Zero position offset (radians)
- **reverse**: Invert encoder reading direction
- **dead_zone_correction_angle**: Angle of The center of a potion the robot can't move to due to mechanical constrains (radians), 0 if not needed.
- **velocity_sample_amount**: Number of samples for velocity estimation
- **enable** (motor only): Enable/disable motor encoder

### PID Controller
- **pid_p**: Proportional gain
- **pid_i**: Integral gain
- **pid_d**: Derivative gain

### Movement Limits
- **max_velocity**: Maximum allowed joint velocity (rad/s)
- **limit_lower / limit_upper**: Joint position limits (radians)
- **control_mode**: Active control mode on startup (see KonArmControlMode)
- **max_acceleration**: Maximum allowed acceleration (rad/s²)

### CAN Communication
- **can_base_id**: Base CAN identifier for the joint module

## Dependencies

- `ament_cmake`: Build system
- `rclcpp`: ROS 2 C++ client library
- `std_msgs`: Standard ROS message types
- `rosidl_default_generators`: Message generation tools

## Building the Package
```bash
colcon build --packages-select konarm_driver_msg
```

## Related Packages

- **konarm_driver**: Main driver implementation using these message definitions



## License

MIT License - see LICENSE file for details.
