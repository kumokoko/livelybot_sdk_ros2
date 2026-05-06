cmake_minimum_required(VERSION 3.8)
project(livelybot_oled)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

find_library(SERIALPORT_LIBRARY serialport)
if(NOT SERIALPORT_LIBRARY)
  message(FATAL_ERROR "libserialport not found")
endif()

add_library(livelybot_oled_core STATIC
  src/ip_addr.cpp
  src/sensor_actuator_status.cpp
)
target_include_directories(livelybot_oled_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(livelybot_oled_core serial)
target_link_libraries(livelybot_oled_core ${SERIALPORT_LIBRARY})

add_executable(livelybot_oled_node src/oled_node.cpp)
target_include_directories(livelybot_oled_node PUBLIC include)
ament_target_dependencies(livelybot_oled_node
  rclcpp
  sensor_msgs
  std_msgs
  tf2
)
target_link_libraries(livelybot_oled_node livelybot_oled_core)

install(TARGETS livelybot_oled_core livelybot_oled_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(livelybot_oled_core)
ament_export_dependencies(rclcpp sensor_msgs serial std_msgs tf2)
ament_package()
