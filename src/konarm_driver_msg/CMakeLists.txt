cmake_minimum_required(VERSION 3.8)
project(konarm_driver_msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/KonarmGetConfig.srv"
  "srv/KonarmSetConfig.srv"
  "msg/KonArmControlMode.msg"
  DEPENDENCIES std_msgs
)


ament_export_dependencies(rosidl_default_generators )

ament_package()
