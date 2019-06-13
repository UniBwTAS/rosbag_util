cmake_minimum_required(VERSION 2.8.3)
project(rosbag_util)

find_package(catkin REQUIRED COMPONENTS rospy tf2 tf2_ros)

catkin_package(
  CATKIN_DEPENDS tf2 tf2_ros
)
