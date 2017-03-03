cmake_minimum_required(VERSION 2.8.3)
project(altimeter_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(altimeter_driver src/altimeter_driver.cpp)

target_link_libraries(altimeter_driver
  ${catkin_LIBRARIES}
)

