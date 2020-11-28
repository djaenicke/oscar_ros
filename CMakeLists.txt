cmake_minimum_required(VERSION 3.0.2)
project(robo_car_ros_if)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslint
  std_msgs
  tf
  oscar_pi
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robo_car_ros_if
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(go_to_point src/go_to_point.cpp)
add_dependencies(go_to_point ${catkin_EXPORTED_TARGETS})

add_executable(velocity_cmd src/velocity_cmd.cpp)
add_dependencies(velocity_cmd ${catkin_EXPORTED_TARGETS})

add_executable(teleop src/teleop.cpp)
add_dependencies(teleop ${catkin_EXPORTED_TARGETS})

add_executable(odometry src/odometry.cpp)
add_dependencies(odometry ${catkin_EXPORTED_TARGETS})

add_executable(simple_nav src/simple_nav.cpp)
add_dependencies(simple_nav ${catkin_EXPORTED_TARGETS})

target_link_libraries(velocity_cmd ${catkin_LIBRARIES})
target_link_libraries(teleop ${catkin_LIBRARIES})
target_link_libraries(odometry ${catkin_LIBRARIES})
target_link_libraries(simple_nav go_to_point ${catkin_LIBRARIES})

set(ROSLINT_CPP_OPTS "--filter=-legal/copyright")
roslint_cpp()
