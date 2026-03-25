cmake_minimum_required(VERSION 3.8)
project(emm_v5_mpc_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

add_executable(mpc_follow_joint_trajectory_server
  src/mpc_follow_joint_trajectory_server.cpp)

ament_target_dependencies(mpc_follow_joint_trajectory_server
  rclcpp
  rclcpp_action
  trajectory_msgs
  sensor_msgs
  control_msgs
  builtin_interfaces)

install(TARGETS
  mpc_follow_joint_trajectory_server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
