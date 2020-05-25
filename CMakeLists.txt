cmake_minimum_required(VERSION 2.8.3)
project(kalman_filter_odometry)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  tf
  nav_msgs
)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kalman_filter_odometry
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs geometry_msgs tf nav_msgs
)

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}_node src/kf_odom_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    myFilesLib
)

ADD_LIBRARY(myFilesLib
    src/kf.cpp
    src/kinematic.cpp
    src/kf_odom_node.cpp
)
