python3 -m venv .venv && source .venv/bin/activate

# git submodule update --init --recursive





python3 -m pip install -r requirements.txt
pre-commit install
pre-commit autoupdate
pre-commit install


source /opt/ros/jazzy/setup.bash

colcon build --symlink-install
source install/setup.bash
source install/local_setup.bash
rosdep install --from-paths src --ignore-src -r -y
rosdep update

colcon build --symlink-install
source install/setup.bash
source install/local_setup.bash
