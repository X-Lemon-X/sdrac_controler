cmake_minimum_required(VERSION 3.8)
project(konarm_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Set required dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  std_msgs
  generate_parameter_library
  sensor_msgs
  ari_shared_types
  can_device
  diagnostic_msgs
  konarm_driver_msg
)

set(CMAKE_CXX_STANDARD 20)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## PARAMETERS
set(PARAMETERS_LIB_NAME "${PROJECT_NAME}_parameters")
generate_parameter_library( ${PARAMETERS_LIB_NAME}
  src/${PARAMETERS_LIB_NAME}.yaml
)

set(EXPORT_COMPILE_COMMANDS ON)


add_executable(konarm_driver
src/konarm_driver.cpp
src/konarm_joint_driver.cpp
)

add_executable(konarm_basic_joystick_driver
src/konarm_basic_joystick_driver.cpp
)

add_subdirectory(sdrac_shared_types)


target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(
  ${PROJECT_NAME} PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(
  ${PROJECT_NAME} PUBLIC
    ${PARAMETERS_LIB_NAME}
    sdrac_shared_types
)

target_include_directories(konarm_basic_joystick_driver PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(
  konarm_basic_joystick_driver PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(
  konarm_basic_joystick_driver PUBLIC
    ${PARAMETERS_LIB_NAME}
    sdrac_shared_types

)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS konarm_basic_joystick_driver
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)


ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
