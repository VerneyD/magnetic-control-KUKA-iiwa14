cmake_minimum_required(VERSION 3.8)
project(test_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)



      # GAZEBO CLASSIC for my_box.cpp
# find_package(gazebo REQUIRED)
# include_directories(${GAZEBO_INCLUDE_DIRS})
# link_directories(${GAZEBO_LIBRARY_DIRS})

# add_library(my_box_plugin SHARED src/my_box_plugin.cpp)
# target_link_libraries(my_box_plugin ${GAZEBO_LIBRARIES})


      # IGNITION for my_box_ign_plugin 
# Plugin Ignition

find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)


add_library(my_box_ign_plugin SHARED src/my_box_ign_plugin.cpp)
target_link_libraries(my_box_ign_plugin
  ignition-gazebo6::core
  ignition-plugin1::ignition-plugin1
)


install(TARGETS my_box_ign_plugin
        LIBRARY DESTINATION lib)

add_library(robot_movement_plugin SHARED src/robot_movement_plugin.cpp)
target_link_libraries(robot_movement_plugin
  ignition-gazebo6::core
  ignition-plugin1::ignition-plugin1
)
ament_target_dependencies(robot_movement_plugin
  rclcpp
  sensor_msgs
)
target_include_directories(robot_movement_plugin PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(TARGETS robot_movement_plugin
        LIBRARY DESTINATION lib)



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

ament_package()
