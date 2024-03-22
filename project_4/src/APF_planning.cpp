/*
Date: 20.12.2023
Developed by: Eyüp Şahin
Project: Navigate robot to the goal position using APF method 
Summary: The robot is navigated to the goal position using APF method. The robot is controlled using the velocities calculated by TurtleBotController::updateVelocity function.
*/ 

#include <fstream>
#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <ros/package.h>

#include "turtlebot_controller.h"
#include "to_euler.h"


geometry_msgs::Point robot_position;
double yaw;
void ModelStatesCb(const gazebo_msgs::ModelStatesConstPtr &msg) {
    for (int i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == "robot") {
            robot_position = msg->pose[i].position;
            yaw = ToEulerAngle(msg->pose[i].orientation).z;

        }
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "apf_planning");
    ros::NodeHandle nh;

    TurtleBotController controller(WHEEL_DIAMETER / 2, WHEEL_SEPERATION);

    ros::Publisher command_velocity_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber model_states_subscriber = nh.subscribe("/gazebo/model_states", 1, ModelStatesCb);

    ros::Rate control_rate(CONTROL_RATE);
    ros::Duration(1).sleep();

    std::vector<geometry_msgs::Point> path;
    geometry_msgs::Twist command_vel;
    Vector3d last_pos = Vector3d::Zero();

    controller.readFile();

    int counter = 0;
    
    while (ros::ok()) {
        
            controller.calcForce(robot_position); // calculate total force acting on the robot
            controller.updateVelocities(command_vel, robot_position, yaw); // update velocities
        if (counter == CONTROL_RATE) {
            counter = 0;
            
            ROS_INFO("Current robot position: %f, %f, %f,  yaw: %f", robot_position.x, robot_position.y, robot_position.z, yaw);
            ROS_INFO("Current robot velocity: %f, %f\n", command_vel.linear.x, command_vel.angular.z);
            path.push_back(robot_position); // add the current location of robot to path
        }
        if(controller.isReached(robot_position)){ // check if the robot has reached the goal and stop it
            command_vel.linear.x = 0;
            command_vel.angular.z = 0;
            command_velocity_publisher.publish(command_vel);
            ros::spinOnce();
            control_rate.sleep();
            break;
        }

        command_velocity_publisher.publish(command_vel);

        counter++;
        ros::spinOnce();
        control_rate.sleep();
    }

    controller.savePath(path);


    return 0;
}
