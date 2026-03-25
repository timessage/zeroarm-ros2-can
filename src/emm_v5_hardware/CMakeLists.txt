cmake_minimum_required(VERSION 3.8)
project(emm_v5_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(std_srvs REQUIRED)
# yaml_cpp_vendor typically exposes yaml-cpp
find_package(yaml-cpp QUIET)
find_package(emm_v5_driver REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/emm_v5_system.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

if(yaml-cpp_FOUND)
  target_compile_definitions(${PROJECT_NAME} PRIVATE EMM_V5_HAVE_YAML_CPP)
  target_link_libraries(${PROJECT_NAME} PRIVATE yaml-cpp)
endif()

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  pluginlib
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  emm_v5_driver
  std_srvs
)

pluginlib_export_plugin_description_file(hardware_interface emm_v5_hardware_plugin.xml)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES emm_v5_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
