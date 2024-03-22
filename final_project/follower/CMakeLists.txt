cmake_minimum_required(VERSION 3.0.2)
project(follower)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)


###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(runfollower src/run_follower.cpp src/follower.cpp)


target_link_libraries(runfollower
  ${catkin_LIBRARIES}
)
