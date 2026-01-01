#!/bin/bash

source /opt/ros/jazzy/setup.bash
source install/setup.bash
source install/local_setup.bash
colcon build --symlink-install
