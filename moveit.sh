source /opt/ros/humble/setup.bash

sudo apt install python3-rosdep

sudo rosdep init
rosdep update
sudo apt update
sudo apt dist-upgrade
sudo apt install ros-humble-gazebo-ros-pkgs

sudo apt install python3-colcon-common-extensions
sudo apt install python3-colcon-mixin
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default

sudo apt install python3-vcstool