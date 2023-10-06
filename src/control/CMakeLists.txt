cmake_minimum_required(VERSION 3.0.2)
project(control)


SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")



find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  common_msgs
  huat_msgs
  nav_msgs
  roscpp
  std_msgs
  eigen_conversions
)
find_package(Eigen3 REQUIRED)

catkin_package(

 CATKIN_DEPENDS geometry_msgs huat_msgs nav_msgs roscpp std_msgs

)




include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(PP_car src/PP_car.cpp)

target_link_libraries(PP_car
  ${catkin_LIBRARIES}
)

