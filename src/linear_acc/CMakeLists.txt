cmake_minimum_required(VERSION 3.0.2)
project(linear_acc)
add_compile_options(-std=c++11)
add_definitions(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
common_msgs
geometry_msgs
nav_msgs
nodelet
pcl_ros
roscpp
sensor_msgs
std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS common_msgs geometry_msgs nav_msgs nodelet pcl_ros roscpp sensor_msgs std_msgs
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_library(${PROJECT_NAME} src/linear_acc.cpp src/linear_acc_nodelet.cc)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
