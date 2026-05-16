cmake_minimum_required(VERSION 3.8)
project(livelybot_logger)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LoggerOperation.msg"
  "msg/LoggerStatus.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(${PROJECT_NAME}_core SHARED src/logger_interface.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_core rclcpp)
target_link_libraries(${PROJECT_NAME}_core "${cpp_typesupport_target}")

add_executable(${PROJECT_NAME}_node src/logger_node.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  std_msgs
  tf2
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core "${cpp_typesupport_target}")

install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(
  builtin_interfaces
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  rosidl_default_runtime
)
ament_package()
