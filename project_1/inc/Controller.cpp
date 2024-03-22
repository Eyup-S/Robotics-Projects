/*
Date: 18.10.2023
Developed by: Eyüp Şahin
Project: EE 451 Project 1
Summary: This class is used to control the robot arm. It has methods to move the arm in x, y and z directions. 
It also has a method to move the arm next to the cube and it has a method to push the cube in the direction of the arm. 
*/

#include "Controller.hpp"

// Constructor
Controller::Controller(){
    //create a node handle
    ros::NodeHandle nh;
    //create arm position publishers
    pubx = nh.advertise<std_msgs::Float64>("/PPP_Robot/x_axis_controller/command", 1000);
    puby = nh.advertise<std_msgs::Float64>("/PPP_Robot/y_axis_controller/command", 1000);
    pubz = nh.advertise<std_msgs::Float64>("/PPP_Robot/z_axis_controller/command", 1000);
    
}

//move the arm in x-axis
void Controller::moveX(double x){
    std_msgs::Float64 msg; //create a message
    //check if the x-axis data is within the workspace
    if (x < X_AXIS_WS_MIN || x > X_AXIS_WS_MAX){
        ROS_WARN_ONCE("The X-axis arm reached end position (%f). Stopping ...",x);
        return;
    }
    msg.data = x; 
    pubx.publish(msg); 

    //wait for arm to reach the target
    ros::Rate loop_rate(5);
    while(ros::ok() && (joint_state.position[0] < msg.data - PROXIMITY_OFFSET 
                        || joint_state.position[0] > msg.data + PROXIMITY_OFFSET ))
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

//move the arm in y-axis
void Controller::moveY(double y){
    //check if the y-axis data is within the workspace
    if (y < Y_AXIS_WS_MIN || y > Y_AXIS_WS_MAX){
        ROS_WARN_ONCE("The Y-axis arm reached end position (%f). Stopping ...",y);
        return;
    }
    std_msgs::Float64 msg; //create a message
    msg.data = y; 
    puby.publish(msg); //publish the message

    //wait for arm to reach the target
    ros::Rate loop_rate(5);
    while(ros::ok() && (joint_state.position[1] < msg.data - PROXIMITY_OFFSET 
                         || joint_state.position[1] > msg.data + PROXIMITY_OFFSET))
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

//move the arm in z-axis
void Controller::moveZ(double z){
    //check if the z-axis data is within the workspace
    if (z < Z_AXIS_WS_MIN || z > Z_AXIS_WS_MAX){
        ROS_WARN_ONCE("The Z-axis arm reached end position (%f). Stopping ...",z);
        return;
    }
    std_msgs::Float64 msg; //create a message
    msg.data = Z_OFFSET + z; //add offset to the z-axis
    pubz.publish(msg); //publish the message

    //wait for arm to reach the target
    ros::Rate loop_rate(5);
    while(ros::ok() && (joint_state.position[2] < msg.data - PROXIMITY_OFFSET 
                        || joint_state.position[2] > msg.data + PROXIMITY_OFFSET ))
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

//randomly generate a number between 1 and 4
int Controller::randomDirection()
{
    
    std::random_device rd; // obtain a random number from random number engine
    std::mt19937 gen(rd()); // seed the generator
    std::uniform_real_distribution<> dis(1.0,4.1); // define the range
    ROS_INFO_ONCE("Random direction: %d",(int)dis(gen));
    return (int)dis(gen); //return the generated number as integer
    
}

bool Controller::goToCube(geometry_msgs::Pose pose, int direction)
{   
    //print position of the cube
    ROS_INFO_ONCE("Cube position: %f, %f", pose.position.x, pose.position.y);

    //push the cube in the direction
    switch (direction)
    {
    case 1:
        moveX(pose.position.x  + GOTOCUBE_OFFSET); //move the arm to the right of the cube
        moveY(pose.position.y);
        moveZ(z_ground_level);
        break; 

    case 2:
        moveX(pose.position.x - GOTOCUBE_OFFSET); //move the arm to the left of the cube
        moveY(pose.position.y);
        moveZ(z_ground_level);
        break;
    case 3:
        moveX(pose.position.x);
        moveY(pose.position.y + GOTOCUBE_OFFSET); //move the arm to the back of the cube
        moveZ(z_ground_level);
        break;
    case 4:
        moveX(pose.position.x);
        moveY(pose.position.y - GOTOCUBE_OFFSET); //move the arm to the front of the cube
        moveZ(z_ground_level);
        break;
    default:
        break;
    }
    //check if the arm is in the correct position
    if(pose.position.x - PROXIMITY_OFFSET  < joint_state.position[0] < pose.position.x + PROXIMITY_OFFSET 
        && pose.position.y - PROXIMITY_OFFSET < joint_state.position[1] == pose.position.y + PROXIMITY_OFFSET 
        && 0.5 - PROXIMITY_OFFSET < joint_state.position[2] < 0.5 + PROXIMITY_OFFSET ){
        return false;
    }
    else{
        return true;
    }
}

//push the cube continuously
void Controller::push(int dir)
{
    switch(dir)
    {
        case 1:
            ROS_INFO_ONCE("Pushing the cube to the left");
            moveX(joint_state.position[0] - PUSH_STEP); //move the arm to the left
            break;
        case 2:
            ROS_INFO_ONCE("Pushing the cube to the right");
            moveX(joint_state.position[0] + PUSH_STEP); //move the arm to the right
            break;
        case 3:
            ROS_INFO_ONCE("Pushing the cube to the front");
            moveY(joint_state.position[1] - PUSH_STEP); //move the arm to the front
            break;
        case 4:
            ROS_INFO_ONCE("Pushing the cube to the back");
            moveY(joint_state.position[1] + PUSH_STEP); //move the arm to the back
            break;
        default:
            break;

    }
}


//joint state callback function
void Controller::jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    joint_state = *msg;
}

//get joint state
sensor_msgs::JointState Controller::getJointState()
{
    return joint_state;
}

