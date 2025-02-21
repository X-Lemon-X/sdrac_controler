python3 -m venv .venv && source .venv/bin/activate

git submodule update --init --recursive
src/sdrac_can_stranslator/sdrac_can_stranslator/ariadna_constants/./generate-files.sh


python3 -m pip install -r requirements.txt

source /opt/ros/humble/setup.bash

colcon build --symlink-install
source install/setup.bash
rosdep install --from-paths src --ignore-src -r -y
rosdep update

colcon build --symlink-install
source install/setup.bash








python3 src/sdrac_can_stranslator/sdrac_can_stranslator/can_device.py
ros2 launch launch_files/develop_rc.launch.py
