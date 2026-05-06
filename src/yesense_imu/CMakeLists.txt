cmake_minimum_required(VERSION 3.8)
project(yesense_imu)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_library(SERIALPORT_LIBRARY serialport)
if(NOT SERIALPORT_LIBRARY)
  message(FATAL_ERROR "libserialport not found")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/YesenseIMUSetting.msg"
  "msg/YesenseImuAllData.msg"
  "msg/YesenseImuCmdResp.msg"
  "msg/YesenseImuEulerAngle.msg"
  "msg/YesenseImuGnssData.msg"
  "msg/YesenseImuGpsData.msg"
  "msg/YesenseImuLocation.msg"
  "msg/YesenseImuMasterGnssData.msg"
  "msg/YesenseImuQuaternion.msg"
  "msg/YesenseImuSensorData.msg"
  "msg/YesenseImuSlaveGnssData.msg"
  "msg/YesenseImuStatus.msg"
  "msg/YesenseImuUtcTime.msg"
  "msg/YesenseImuVelocity.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(yesense_imu_node
  src/analysis_data.cpp
  src/yesense_minimal_node.cpp
)
target_include_directories(yesense_imu_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
ament_target_dependencies(yesense_imu_node
  geometry_msgs
  rclcpp
  sensor_msgs
  serial
  std_msgs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(yesense_imu_node ${SERIALPORT_LIBRARY} "${cpp_typesupport_target}")

install(TARGETS yesense_imu_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY cfg launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(geometry_msgs rclcpp sensor_msgs serial std_msgs tf2 tf2_geometry_msgs rosidl_default_runtime)
ament_package()
