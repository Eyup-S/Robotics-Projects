cmake_minimum_required(VERSION 3.0.2)
project(rrrbot)



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


add_executable(random_kinematics src/random_kinematics.cpp include/kinematics.h src/kinematics.cpp)
add_executable(show_workspace src/show_workspace.cpp include/kinematics.h src/kinematics.cpp)
add_executable(touch_spheres src/touch_spheres.cpp include/kinematics.h src/kinematics.cpp)
target_link_libraries(random_kinematics ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(show_workspace ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(touch_spheres ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
