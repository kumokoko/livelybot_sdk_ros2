cmake_minimum_required(VERSION 3.8)
project(livelybot_power)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PowerDetect.msg"
  "msg/PowerSwitch.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

add_library(livelybot_can_power STATIC
  src/livelybot_can_driver.cpp
)
target_include_directories(livelybot_can_power PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(livelybot_can_power pthread)

add_executable(power_node src/power_class.cpp)
target_include_directories(power_node PUBLIC include)
ament_target_dependencies(power_node rclcpp std_msgs)
rosidl_target_interfaces(power_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(power_node livelybot_can_power)

install(TARGETS livelybot_can_power power_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(livelybot_can_power)
ament_export_dependencies(rosidl_default_runtime rclcpp std_msgs sensor_msgs)
ament_package()
