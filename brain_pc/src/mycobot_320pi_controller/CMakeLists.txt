cmake_minimum_required(VERSION 3.5)
project(mycobot_320pi_controller)

# Find all required packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)  # Add this - it was missing

find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

# Slider control executable
add_executable(slider_control src/slider_control.cpp)
ament_target_dependencies(slider_control 
  rclcpp 
  trajectory_msgs 
  sensor_msgs
)

# Main controller library
add_library(${PROJECT_NAME} SHARED 
  src/mycobot_320pi_interface.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE 
  include
  "${SERIAL_INCLUDE_DIRS}"
)

target_link_libraries(${PROJECT_NAME}
 "${SERIAL_LDFLAGS}")

# Dependencies for main controller
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

pluginlib_export_plugin_description_file(hardware_interface mycobot_320pi_controller.xml)

ament_python_install_package(${PROJECT_NAME})

# Install targets
install(TARGETS
  slider_control
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/slider_control.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

# ----------------------------------------------------------
# Install python scripts correctly
# ----------------------------------------------------------
install(PROGRAMS
  scripts/arm_constants.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

ament_package()