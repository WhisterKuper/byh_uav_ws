cmake_minimum_required(VERSION 2.8.3)
# byh_uav_uart
project(byh_uav_uart)

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
  byh_uav
)

add_compile_options(-g -std=c++11)  

# BOOST 库
find_package(Boost REQUIRED COMPONENTS system)

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# 头文件路径
include_directories( 
  ${OpenCV_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS}
  ~/Ros/byh_uav_ws/src/byh_uav_uart/include
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime geometry_msgs
)

# 执行文件
add_executable(byh_uav_uart src/byh_uav.cpp src/quaternion.cpp)
target_link_libraries(byh_uav_uart ${catkin_LIBRARIES})
