cmake_minimum_required(VERSION 3.8)
project(livelybot_serial)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(livelybot_interfaces REQUIRED)
find_package(Threads REQUIRED)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

find_library(SERIALPORT_LIBRARY serialport)
if(NOT SERIALPORT_LIBRARY)
  message(FATAL_ERROR "libserialport not found")
endif()

set(serial_sources
  src/lively_serial.cc
  src/hardware/motor.cc
  src/hardware/canport.cc
  src/hardware/canboard.cc
  src/hardware/robot.cc
  src/crc/crc8.cc
  src/crc/crc16.cc
)

add_library(${PROJECT_NAME} SHARED ${serial_sources})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  serial
)
target_link_libraries(${PROJECT_NAME} rt Threads::Threads ${SERIALPORT_LIBRARY})

add_executable(motor_driver_node src/motor_driver_node.cpp)
target_compile_features(motor_driver_node PRIVATE cxx_std_17)
target_include_directories(motor_driver_node PRIVATE
  include
)
ament_target_dependencies(motor_driver_node
  livelybot_interfaces
  rclcpp
  sensor_msgs
  std_msgs
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

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp sensor_msgs std_msgs serial)

if(BUILD_TESTING)
  ament_add_gtest(test_runtime_config test/test_runtime_config.cpp)
  target_compile_features(test_runtime_config PRIVATE cxx_std_17)
  target_include_directories(test_runtime_config PRIVATE include)
  ament_target_dependencies(test_runtime_config rclcpp sensor_msgs serial)
  target_link_libraries(test_runtime_config ${PROJECT_NAME})

  ament_add_gtest(test_motor_commands test/test_motor_commands.cpp)
  target_compile_features(test_motor_commands PRIVATE cxx_std_17)
  target_include_directories(test_motor_commands PRIVATE include)
  ament_target_dependencies(test_motor_commands rclcpp sensor_msgs serial)
  target_link_libraries(test_motor_commands ${PROJECT_NAME})
endif()

ament_package()
