python3 -m venv .venv && source .venv/bin/activate
source /opt/ros/humble/setup.bash
source install/local_setup.bash 
ros2 launch launch_files/nomad_rc.launch.py
