cmake_minimum_required(VERSION 2.8.3)
project(mplan_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Pose2DStamped.msg
  Twist2DStamped.msg
  Vector2D.msg
  MovingObject.msg
  Actor.msg
  Obstacles.msg
  TimedPath.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES obst_avoid_msgs
   CATKIN_DEPENDS
   geometry_msgs
   sensor_msgs
   std_msgs
   message_runtime
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
