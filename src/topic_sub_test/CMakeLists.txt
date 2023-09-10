cmake_minimum_required(VERSION 3.0.2)
project(topic_sub_test)

find_package(catkin REQUIRED COMPONENTS
pcl_ros
roscpp
sensor_msgs
geometry_msgs
common_msgs
std_msgs
nav_msgs
nodelet
)
find_package(Threads REQUIRED)

add_message_files(
  FILES
  Cone.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(cone_sub src/cone_sub.cpp)
# add_dependencies(cone_sub ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(cone_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(cone_sub
${catkin_LIBRARIES}
)


catkin_package(
 CATKIN_DEPENDS roscpp  std_msgs message_runtime
)
