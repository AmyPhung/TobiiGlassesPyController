cmake_minimum_required(VERSION 3.0.2)
project(tobii_glasses_ros)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
