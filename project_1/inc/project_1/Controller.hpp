#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <random>

#define X_AXIS_WS_MIN 0 // X-axis workspace minimum value
#define X_AXIS_WS_MAX 1 // X-axis workspace maximum value
#define Y_AXIS_WS_MIN 0 // Y-axis workspace minimum value
#define Y_AXIS_WS_MAX 1 // Y-axis workspace maximum value
#define Z_AXIS_WS_MIN -0.3 // Z-axis workspace minimum value
#define Z_AXIS_WS_MAX 0.5 // Z-axis workspace maximum value
#define PROXIMITY_OFFSET 0.01 // proximity offset for joint state
#define GOTOCUBE_OFFSET 0.1 // distance between the cube and the robot grip
#define Z_OFFSET -0.3 // offset for z-axis
#define PUSH_STEP 0.1 // step size for pushing the cube

class Controller {
    public:

    Controller();
    void moveX(double x); //move the arm in x-axis
    void moveY(double y); //move the arm in y-axis
    void moveZ(double z); //move the arm in z-axis
    bool goToCube(geometry_msgs::Pose,int); //move the arm next to the cube
    void push(int); //push the cube in the direction of the arm
    int randomDirection(); //randomly select a direction
    void jointCallback(const sensor_msgs::JointState::ConstPtr&); //callback function for joint state
    sensor_msgs::JointState getJointState(); //get the joint state
    

    private:
    ros::Publisher pubx; //publisher for x-axis
    ros::Publisher puby; //publisher for y-axis
    ros::Publisher pubz; //publisher for z-axis
    sensor_msgs::JointState joint_state; //message to store joint state
    double z_ground_level = 0.05; //z-axis ground level    
};

#endif