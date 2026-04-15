#!/bin/bash
source /opt/ros/jazzy/setup.bash

python3 -m venv .venv && source .venv/bin/activate
python3 -m pip install -r requirements.txt
pre-commit install
pre-commit autoupdate
pre-commit install

rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
