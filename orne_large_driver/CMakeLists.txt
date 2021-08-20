cmake_minimum_required(VERSION 2.8.3)
project(orne_large_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  message_generation
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

include_directories(
 include ${catkin_INCLUDE_DIRS}
)

add_executable(orne_large_driver src/orne_large_driver.cpp src/blm.cpp)
add_executable(orne_large_driver_with_arduino src/orne_large_driver_with_arduino.cpp)

target_link_libraries(orne_large_driver ${catkin_LIBRARIES} usb ftdi)
target_link_libraries(orne_large_driver_with_arduino ${catkin_LIBRARIES})

