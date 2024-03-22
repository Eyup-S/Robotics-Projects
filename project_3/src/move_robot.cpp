#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Point.h>

#include "velocity_kinematics.h"

//get the joint states of the robot
sensor_msgs::JointState joint_states;
void JointStatesCb(const sensor_msgs::JointStateConstPtr &msg) {
    joint_states = *msg;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "move_robot");
    ros::NodeHandle nh;

    //set the publishers and subscribers
    ros::Publisher joint1_publisher = nh.advertise<std_msgs::Float64>("/rrrbot/joint1_controller/command", 10);
    ros::Publisher joint2_publisher = nh.advertise<std_msgs::Float64>("/rrrbot/joint2_controller/command", 10);
    ros::Publisher joint3_publisher = nh.advertise<std_msgs::Float64>("/rrrbot/joint3_controller/command", 10);

    ros::Subscriber joint_states_subscriber = nh.subscribe("/rrrbot/joint_states", 1, JointStatesCb);
    

    
    ros::Rate control_rate(100);
    ros::Duration(1).sleep();

    //set the initial joint values
    std_msgs::Float64 joint1_command_value, joint2_command_value, joint3_command_value;
    joint1_command_value.data = 0;
    joint2_command_value.data = 0;
    joint3_command_value.data = 0;

    VelocityKinematics velocity_kinematics;

    int state_counter = 0;

    //set the initial state of the robot and give it some time to reach the initial state
    while(ros::ok()){
        state_counter++;
        if (state_counter == 100)
            break;
        joint1_publisher.publish(joint1_command_value);
        joint2_publisher.publish(joint2_command_value);
        joint3_publisher.publish(joint3_command_value);
        ros::spinOnce();
        control_rate.sleep();

    }
    std::vector<double> thetas;
    while(ros::ok()){
        thetas = velocity_kinematics.run();
        joint1_command_value.data = thetas[0];
        joint2_command_value.data = thetas[1];
        joint3_command_value.data = thetas[2];

        joint1_publisher.publish(joint1_command_value);
        joint2_publisher.publish(joint2_command_value);
        joint3_publisher.publish(joint3_command_value);
        
        if(velocity_kinematics.d_t == 600) break;
        ros::spinOnce();
        control_rate.sleep();

    }


    return 0;
}