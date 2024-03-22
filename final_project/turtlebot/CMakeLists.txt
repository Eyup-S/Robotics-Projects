cmake_minimum_required(VERSION 3.0.2)
project(turtlebot)



find_package(catkin REQUIRED COMPONENTS
        roscpp
        gazebo_ros
        gazebo_ros_control
        gazebo_msgs
        controller_manager
        effort_controllers
        robot_state_publisher
        std_msgs
        geometry_msgs
        sensor_msgs
        )
        

find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})



list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp gazebo_ros gazebo_ros_control controller_manager effort_controllers robot_state_publisher std_msgs sensor_msgs
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)


add_executable(movements src/movements.cpp include/geometric.h src/geometric.cpp)
target_link_libraries(movements ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

