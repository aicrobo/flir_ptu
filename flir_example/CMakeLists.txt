cmake_minimum_required(VERSION 2.8.3)
project(flir_example)

find_package(catkin REQUIRED COMPONENTS roscpp rospy)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aicroboxi_example
  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ptu_move nodes/ptu_move.cpp)
target_link_libraries(ptu_move ${catkin_LIBRARIES})