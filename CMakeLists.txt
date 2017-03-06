cmake_minimum_required(VERSION 2.8.3)
project(altimeter_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)


catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
    message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-std=c++11)


add_executable(altimeter_driver src/altimeter_driver.cpp src/basic_serial.cpp include/basic_serial.h)

target_link_libraries(altimeter_driver
  ${catkin_LIBRARIES}
)

add_dependencies(altimeter_driver ${catkin_EXPORTED_TARGETS})

