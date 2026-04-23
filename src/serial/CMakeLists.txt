cmake_minimum_required(VERSION 3.8)
project(serial)

find_package(ament_cmake REQUIRED)

set(serial_SRCS
  src/serial.cc
  src/impl/unix.cc
  src/impl/list_ports/list_ports_linux.cc
)

add_library(${PROJECT_NAME} SHARED ${serial_SRCS})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} rt pthread)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
