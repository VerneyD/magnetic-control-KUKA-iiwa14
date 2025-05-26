cmake_minimum_required(VERSION 3.8)
project(my_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED) 
# Trouver les dépendances Python
find_package(rclpy REQUIRED)

# Installer les scripts Python
install(
  PROGRAMS
  src/position_control_gui.py
  DESTINATION lib/${PROJECT_NAME}
)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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


add_executable(command_publisher2 src/command_publisher2.cpp)
ament_target_dependencies(command_publisher2 rclcpp sensor_msgs geometry_msgs)
add_executable(command_listener src/command_listener.cpp)
ament_target_dependencies(command_listener rclcpp sensor_msgs geometry_msgs)
add_executable(command_publisher3 src/command_publisher3.cpp)
ament_target_dependencies(command_publisher3 rclcpp sensor_msgs geometry_msgs control_msgs)
add_executable(reset_position src/reset_position.cpp)
ament_target_dependencies(reset_position rclcpp trajectory_msgs std_msgs sensor_msgs)
add_executable(robot_info src/robot_info.cpp)
ament_target_dependencies(robot_info rclcpp trajectory_msgs std_msgs geometry_msgs control_msgs)
add_executable(robot_info_controller src/robot_info_controller.cpp)
ament_target_dependencies(robot_info_controller rclcpp trajectory_msgs std_msgs geometry_msgs sensor_msgs)


find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(end_effector_position_node src/end_effector_position_node.cpp)
ament_target_dependencies(end_effector_position_node  rclcpp  tf2_ros  geometry_msgs  tf2_geometry_msgs)

add_executable(position_control src/position_control.cpp)
ament_target_dependencies(position_control rclcpp trajectory_msgs std_msgs geometry_msgs sensor_msgs control_msgs)

install(TARGETS
  command_publisher2
  command_listener
  command_publisher3
  reset_position
  robot_info
  robot_info_controller
  end_effector_position_node
  position_control
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
