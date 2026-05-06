cmake_minimum_required(VERSION 3.8)
project(livelybot_bringup)

find_package(ament_cmake REQUIRED)
find_package(livelybot_power REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(robot_emergency_stop_node src/robot_emergency_stop_ros2.cpp)
ament_target_dependencies(robot_emergency_stop_node
  livelybot_power
  rclcpp
  sensor_msgs
)

add_executable(robot_falldown_protect_node src/robot_falldown_protect_ros2.cpp)
ament_target_dependencies(robot_falldown_protect_node
  livelybot_power
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(robot_falldown_protect_node yaml-cpp)

install(TARGETS
  robot_emergency_stop_node
  robot_falldown_protect_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
