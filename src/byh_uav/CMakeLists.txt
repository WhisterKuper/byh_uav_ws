cmake_minimum_required(VERSION 2.8.3)
# byh_uav
project(byh_uav)

# 依赖包
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  message_runtime
  nav_msgs
  nav_msgs
  roscpp
  serial
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf
  serial
)

add_compile_options(-g -std=c++11)  

# BOOST 库
find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  FILES
  uav_imu.msg
  uav_gps.msg
  uav_magnet.msg
  uav_barometer.msg
  uav_cmd_frequence.msg
  uav_frequence.msg
  uav_command.msg
  uav_pps.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# 头文件路径
include_directories( 
  ${OpenCV_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS}
  ~/Ros/byh_uav_ws/src/byh_uav/include
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime geometry_msgs
)
