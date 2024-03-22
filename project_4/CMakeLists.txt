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


add_executable(calculate_location src/calculate_location.cpp src/turtlebot_controller.cpp src/to_euler.cpp)
target_link_libraries(calculate_location ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(apf_planning src/APF_planning.cpp src/turtlebot_controller.cpp src/to_euler.cpp)
target_link_libraries(apf_planning ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
