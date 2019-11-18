cmake_minimum_required(VERSION 2.8.3)
project(turtleTutorial)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  turtlesim
)

catkin_package()


include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(walker src/walker.cpp)
target_link_libraries(walker ${catkin_LIBRARIES})


