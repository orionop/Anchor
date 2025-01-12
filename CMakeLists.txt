cmake_minimum_required(VERSION 3.0.2)
project(autonomous_quadcopter_landing_system)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  gazebo_ros
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(control_node src/control_node.py)
target_link_libraries(control_node ${catkin_LIBRARIES})
