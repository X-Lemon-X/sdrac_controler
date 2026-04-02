# 6 DOF ARM-Manipulator Controler
Repo with all the required code to control the 6 DOF arm-manipulator.


### Quick Start
1. Build the workspace:
```bash
colcon build --symlink-install
```
2. Source the workspace:
```bash
source install/setup.bash
```
3. Launch the driver:
```bash
ros2 launch konarm_bringup develop_rc.launch.py
```

### Launch Arguments
You can pass the `use_sim_hardware` parameter to the launch file:

**Example: Use simulated hardware**
```bash
ros2 launch konarm_bringup develop_rc.launch.py use_sim_hardware:=true
```

**Default (real hardware):**
```bash
ros2 launch konarm_bringup develop_rc.launch.py
```
### Base launch file
The base launch file is `develop_rc.launch.py` located in `src/konarm_bringup/launch/`. It includes the:
- `konarm_driver` node for the arm controller.
- `konarm_state_publisher` node for  urdf, TF and joint state publishing.
- `konarm_basic_joystick_driver` node for 6 axis joystick controller (only shouel be used when directly controlling the robot with a joystick, not recommended for use with MoveIt or other high level planners).
- `remote_6d` node responsible for connecting with the remote joystick controllers.


## Parameters
can also be set using the config gile in `src/konarm_bringup/config/konarm_driver_parameters.yaml`
| Parameter Name | Type | Default Value | Description |
|----------------|------|---------------|-------------|
| `frame_id` | string | `"konarm"` | TF frame ID for published joint states |
| `can_interface` | string | `"can0"` | CAN interface name (e.g., can0, vcan0) |
| `control_loop_hz` | double | `75.0` | Control loop frequency in Hz |
| `error_get_hz` | double | `2.0` | Frequency for requesting error frames from joints |
| `timeout_s` | double | `0.5` | Connection timeout in seconds |
| `msg_timeout` | double | `0.5` | Control message timeout in seconds |
| `use_sim_time` | bool | `false` | **Enable simulation mode** (no hardware required) |
| `use_sim_hardware` | bool | `false` | **Enable simulated hardware mode** (no physical CAN interface required) |


