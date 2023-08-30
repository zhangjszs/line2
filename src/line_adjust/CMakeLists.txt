cmake_minimum_required(VERSION 3.0.2)
project(line_adjust)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  tf2
  tf2_geometry_msgs
  tf2_ros
  huat_msgs
)




catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES line_adjust
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

 add_executable(main src/main.cpp)


 target_link_libraries(main
   ${catkin_LIBRARIES}
 )
