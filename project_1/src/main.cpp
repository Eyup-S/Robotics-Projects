/*
Date: 18.10.2023
Developed by: Eyüp Şahin
Project: EE 451 Project 1
Summary: This program spawns a cube in a random position and robot arms push the cube in a random direction.
*/

#include <iostream>
#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <fstream>
#include <ros/package.h>
#include "CubeSpawner.hpp"
#include "Controller.hpp"


int main(int argc, char** argv){

    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;

    std::string path = ros::package::getPath("project_1") + "/urdf/cube.urdf";
    ros::ServiceClient model_spawner_client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    
    CubeSpawner cubeSpawner(model_spawner_client, path);
    std::string cube_name = "cube";
    if (!cubeSpawner.spawn(cube_name))
        return -1;

    //subscribe to model "cube" and get its position
    ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 1000, &CubeSpawner::callback, &cubeSpawner);

    //create a controller object
    Controller controller;
    int randomDir = controller.randomDirection(); //randomly select a direction

    //subscribe to joint states
    ros::Subscriber joint_position = nh.subscribe("/PPP_Robot/joint_states", 1000, &Controller::jointCallback, &controller);
    
    //run spin::once for 2 seconds to get topic messages appropriately
    ros::Rate loop_rate(5);
    for(int i = 0; i < 10; i++)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    sensor_msgs::JointState joint_state = controller.getJointState(); //get the joint state
    geometry_msgs::Pose cubePose = cubeSpawner.getPose(); //get the pose of the cube

    //move the robot to initial position
    controller.moveX(X_AXIS_WS_MAX);
    controller.moveY(Y_AXIS_WS_MAX);
    controller.moveZ(Z_AXIS_WS_MAX);

    //move the arm to next to the cube
    controller.goToCube(cubePose, randomDir);

    //print the joint positions to answer the question b
    ROS_INFO_ONCE("Position of grip: (%f, %f, %f)", joint_state.position[0], joint_state.position[1], joint_state.position[2]);
         
    //push the cube
    while(ros::ok())
    {
        controller.push(randomDir);
        //check if the robot arms reached the maximum workspace
        // if(joint_state.position[0] < X_AXIS_WS_MIN || joint_state.position[0] > X_AXIS_WS_MAX 
        //     || joint_state.position[1] < Y_AXIS_WS_MIN || joint_state.position[1] > Y_AXIS_WS_MAX
        //     || joint_state.position[2]< Z_AXIS_WS_MIN || joint_state.position[2] > Z_AXIS_WS_MAX)
        // {
        //     ROS_INFO_ONCE("Robot reached the maximum workspace. Ending the program ...");
        //     break;
        // }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}