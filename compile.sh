#!/bin/bash

src/ros_can_device/can_constants/./generate-files.sh -v

source /opt/ros/jazzy/setup.bash
source install/setup.bash
source install/local_setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
