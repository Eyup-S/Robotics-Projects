cmake_minimum_required(VERSION 3.0.2)
project(project3)


set(CMAKE_CXX_STANDARD 11)
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
find_package(PythonLibs 3.8 REQUIRED)
include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(/usr/include/python3.8/numpy)



list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp gazebo_ros gazebo_ros_control controller_manager effort_controllers robot_state_publisher std_msgs sensor_msgs
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(move_robot src/move_robot.cpp include/velocity_kinematics.h src/velocity_kinematics.cpp)
# add_executable(plot src/plotter.cpp include/velocity_kinematics.h include/matplotlibcpp.h src/velocity_kinematics.cpp)

target_link_libraries(move_robot ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
# target_link_libraries(plot ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})