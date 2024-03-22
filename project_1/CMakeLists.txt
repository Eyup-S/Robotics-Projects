cmake_minimum_required(VERSION 3.0.2)
project(project_1)

#set minimum cmake version c++11
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
  gazebo_msgs
)

include_directories(
  inc/project_1
  ${catkin_INCLUDE_DIRS}
)

add_executable(PPP_Controller src/main.cpp inc/Controller.cpp inc/CubeSpawner.cpp)


target_link_libraries(PPP_Controller ${catkin_LIBRARIES})
