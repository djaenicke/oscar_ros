cmake_minimum_required(VERSION 3.0.2)
project(robo_car_if)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslint
  std_msgs
  message_generation
  tf
)

add_message_files(
  FILES
  state.msg
  cmd.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robo_car_if
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(go_to_point src/go_to_point.cpp)

add_executable(teleop src/teleop.cpp)
add_executable(pose_estimation src/pose_estimation.cpp)
add_executable(simple_nav src/simple_nav.cpp)

target_link_libraries(teleop ${catkin_LIBRARIES})
target_link_libraries(pose_estimation ${catkin_LIBRARIES})
target_link_libraries(simple_nav go_to_point ${catkin_LIBRARIES})

set(ROSLINT_CPP_OPTS "--filter=-legal/copyright")
roslint_cpp()
