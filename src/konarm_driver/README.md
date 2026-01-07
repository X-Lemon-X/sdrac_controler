# konarm_driver

ROS 2 driver node for KonARM 6-DOF robotic arm with CAN bus communication and simulation support.

For one of two open-source manipulators with CAN bus communication and 6 degrees of freedom.
With reasonable load capacity and reach.
### Sdrac
[SDRAC - 6DOF ](https://github.com/X-Lemon-X/SDRAC_V2)
- 750 mm reach
- 3kg max payload
- Weight: 14kg
- Infinite rotation wrist joints
- 1000W power supply (24V, 40A)
![KonARM Robot Assembly](https://raw.githubusercontent.com/X-Lemon-X/sdrac_mechanical/refs/heads/main/images/Robot_asembly.png){: width="50%"}
### KonARM
[CHWYTATRON - 6DOF]
- 1500 mm reach
- 15kg max payload
- Weight: 20kg (without battery)
- Infinite rotation wrist joints
- Automatic Tool changer
- 10kW peak, best used with 12S4P Li-ion battery pack
![CHWYTATRON](img/Chwytatron%20Z.png){: width="50%"}

**Version:** 0.1.0
**Maintainer:** Patryk Dudziński (dodpat02@gmail.com)
**License:** MIT

## Features

- 6-DOF joint control with position, velocity, and torque modes
- CAN bus communication and simulation mode
- Real-time diagnostics and error reporting
- Runtime configuration services
- Emergency stop support

## ROS 2 Interface

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `joint_states` | `sensor_msgs/msg/JointState` | Current joint positions, velocities, and efforts|
| `diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | System diagnostics and error states |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `joint_control` | `sensor_msgs/msg/JointState` | Joint position/velocity/torque commands |
| `effector_control` | `std_msgs/msg/Int8` | End-effector control (0-100%) |
| `emergency_stop` | `std_msgs/msg/Bool` | Emergency stop trigger (true=stop, false=release) |
| `control_mode` | `konarm_driver_msg/msg/KonArmControlMode` | Set control mode for all joints |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `konarm_get_config` | `konarm_driver_msg/srv/KonarmGetConfig` | Retrieve configuration for a specific joint |
| `konarm_set_config` | `konarm_driver_msg/srv/KonarmSetConfig` | Update configuration for a specific joint |

## Parameters

All parameters can be configured via YAML files or command line. The driver uses `generate_parameter_library` for type-safe parameter handling.

### Core Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `frame_id` | string | `"konarm"` | TF frame ID for published joint states |
| `can_interface` | string | `"can0"` | CAN interface name (e.g., can0, vcan0) |
| `control_loop_hz` | double | `75.0` | Control loop frequency in Hz |
| `error_get_hz` | double | `2.0` | Frequency for requesting error frames from joints |
| `timeout_s` | double | `0.5` | Connection timeout in seconds |
| `msg_timeout` | double | `0.5` | Control message timeout in seconds |
| `use_sim_time` | bool | `false` | **Enable simulation mode** (no hardware required) |

### Parameter File Example

Create a parameter file `konarm_params.yaml`:

```yaml
konarm_driver:
  ros__parameters:
    frame_id: "konarm_base"
    can_interface: "can0"
    control_loop_hz: 100.0
    error_get_hz: 5.0
    timeout_s: 1.0
    msg_timeout: 0.5
    use_sim_time: false  # Set to true for simulation
```

## Simulation Mode

The driver includes a built-in simulation mode that allows testing and development without physical hardware.

### Enabling Simulation

Set the `use_sim_time` parameter to `true`:

```bash
ros2 run konarm_driver konarm_driver --ros-args -p use_sim_time:=true
```

Or in a launch file:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='konarm_driver',
            executable='konarm_driver',
            name='konarm_driver',
            parameters=[{
                'use_sim_time': True,
                'control_loop_hz': 50.0,
                'frame_id': 'konarm'
            }]
        )
    ])
```



## Building

### Dependencies
- ROS 2 (Jazzy or later) other are untested

### CAN Interface Issues

```bash
# Check CAN interface status
ip link show can0

# Check for CAN errors
candump can0

# Reset CAN interface
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

### Driver Not Connecting

1. Verify CAN interface is up and configured
2. Check CAN bitrate matches hardware (typically 1 Mbps)
3. Verify joint modules are powered
4. Check `timeout_s` parameter if using slow hardware
5. Review diagnostics: `ros2 topic echo /diagnostics`

### Simulation Not Working

1. Ensure `use_sim_time` parameter is set to `true`
2. Check that joint commands are being published
3. Verify control loop frequency is reasonable (10-100 Hz)

### Performance Issues

- Lower `control_loop_hz` (default 75 Hz)
- Increase `msg_timeout` for slower networks
- Check CPU usage with `top` or `htop`
- Reduce `error_get_hz` for less diagnostic overhead
