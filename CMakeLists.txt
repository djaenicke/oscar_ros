cmake_minimum_required(VERSION 3.0.2)
project(robo_car_ros_if)

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
  LIBRARIES robo_car_ros_if
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(go_to_point src/go_to_point.cpp)
add_dependencies(go_to_point robo_car_ros_if_generate_messages_cpp)

add_executable(teleop src/teleop.cpp)
add_dependencies(teleop robo_car_ros_if_generate_messages_cpp)

add_executable(slam src/slam.cpp)
add_dependencies(slam robo_car_ros_if_generate_messages_cpp)

add_executable(simple_nav src/simple_nav.cpp)
add_dependencies(simple_nav robo_car_ros_if_generate_messages_cpp)

target_link_libraries(teleop ${catkin_LIBRARIES})
target_link_libraries(slam ${catkin_LIBRARIES})
target_link_libraries(simple_nav go_to_point ${catkin_LIBRARIES})

set(ROSLINT_CPP_OPTS "--filter=-legal/copyright")
roslint_cpp()
