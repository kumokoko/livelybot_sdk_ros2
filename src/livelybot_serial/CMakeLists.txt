cmake_minimum_required(VERSION 3.8)
project(livelybot_serial)

find_package(ament_cmake REQUIRED)
find_package(livelybot_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)

find_library(SERIALPORT_LIBRARY serialport)
if(NOT SERIALPORT_LIBRARY)
  message(FATAL_ERROR "libserialport not found")
endif()

set(serial_SRCS
  src/ros2_compat.cpp
  src/lively_serial.cc
  src/hardware/motor.cc
  src/hardware/canport.cc
  src/hardware/canboard.cc
  src/hardware/robot.cc
  src/crc/crc8.cc
  src/crc/crc16.cc
)

add_library(${PROJECT_NAME} SHARED ${serial_SRCS})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/crc>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/hardware>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  livelybot_interfaces
  rclcpp
  sensor_msgs
  serial
)
target_link_libraries(${PROJECT_NAME} rt pthread ${SERIALPORT_LIBRARY})

add_executable(motor_driver_node src/motor_driver_node.cpp)
target_include_directories(motor_driver_node PUBLIC
  include
  include/crc
  include/hardware
)
ament_target_dependencies(motor_driver_node
  livelybot_interfaces
  rclcpp
  sensor_msgs
  serial
)
target_link_libraries(motor_driver_node ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} motor_driver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY cfg/
  DESTINATION share/${PROJECT_NAME}/cfg
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(livelybot_interfaces rclcpp sensor_msgs serial)
ament_package()
