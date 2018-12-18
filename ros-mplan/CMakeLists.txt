cmake_minimum_required(VERSION 2.8.3)
project(duckietown_mplan)

find_package(catkin REQUIRED COMPONENTS
  rospy
  mplan_msgs
  std_msgs
)


catkin_python_setup()


catkin_package(
  CATKIN_DEPENDS
  std_msgs
  geometry_msgs
  sensor_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
