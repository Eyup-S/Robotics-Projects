/*
Date: 20.12.2023
Developed by: Eyüp Şahin
Project: Estimate the location of the robot based on its wheel velocities
Summary: Using calcPosChange the change of position from the last position is calculated and then added the last position.
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

double left_wheel_velocity, right_wheel_velocity;

geometry_msgs::Point robot_position;
geometry_msgs::Quaternion robot_orientation;
double yaw;
void ModelStatesCb(const gazebo_msgs::ModelStatesConstPtr &msg) {
    for (int i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == "robot") {
            robot_position = msg->pose[i].position;
            robot_orientation = msg->pose[i].orientation;
            yaw = ToEulerAngle(msg->pose[i].orientation).z;

        }
    }
}

void LinkStatesCb(const gazebo_msgs::LinkStatesConstPtr &msg)
{
    for (int i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == "robot::left_wheel_link") {
            left_wheel_velocity = sqrt(msg->twist[i].angular.x * msg->twist[i].angular.x + msg->twist[i].angular.y * msg->twist[i].angular.y);
        } else if (msg->name[i] == "robot::right_wheel_link") {
            right_wheel_velocity = sqrt(msg->twist[i].angular.x * msg->twist[i].angular.x + msg->twist[i].angular.y * msg->twist[i].angular.y);
        } 
    }
}



int main(int argc, char **argv) {

    if(argc != 3){
        std::cout << "Usage: rosrun turtlebot calculate_location <linear velocity> <angular velocity>" << std::endl;
        return 1;
    }
    else if(std::stod(argv[1]) > MAX_LINEAR_VELOCITY || std::stod(argv[1]) < MIN_LINEAR_VELOCITY){
        std::cout << "Linear velocity must be between " << MIN_LINEAR_VELOCITY << " and " << MAX_LINEAR_VELOCITY << std::endl;
        return 1;
    }
    else if(std::stod(argv[2]) > MAX_ANGULAR_VELOCITY || std::stod(argv[2]) < MIN_ANGULAR_VELOCITY){
        std::cout << "Angular velocity must be between " << MIN_ANGULAR_VELOCITY << " and " << MAX_ANGULAR_VELOCITY << std::endl;
        return 1;
    }

    ros::init(argc, argv, "calculate_location");
    ros::NodeHandle nh;

    TurtleBotController controller(WHEEL_DIAMETER / 2, WHEEL_SEPERATION);

    ros::Publisher command_velocity_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber link_states_subscriber = nh.subscribe("/gazebo/link_states", 1, LinkStatesCb);
    ros::Subscriber model_states_subscriber = nh.subscribe("/gazebo/model_states", 1, ModelStatesCb);

    ros::Rate control_rate(CONTROL_RATE);
    geometry_msgs::Twist command_vel;
    Vector3d last_pos = Vector3d::Zero(); // last position is initialized as zero

    int counter = 0;
    ros::Duration(1).sleep();

    while (ros::ok()) {
        
            command_vel.linear.x = std::stod(argv[1]); // user input 1 is given as linear velocity
            command_vel.angular.z = std::stod(argv[2]); // user input 2 is given as angular velocity
            Vector3d pos = controller.calcPosChange(last_pos(2), left_wheel_velocity, right_wheel_velocity); // calculates the change of position from the last position
            last_pos += pos; // adds the change of position to the last position
            if (counter == CONTROL_RATE) { // prints the position and velocity information every second

                // yaw is between -pi and pi, if it is out of this range, it is adjusted
                if(last_pos(2) > M_PI){
                    last_pos(2) -= 2 * M_PI;
                }
                else if(last_pos(2) < -M_PI){
                    last_pos(2) += 2 * M_PI;
                }
                counter = 0;
                ROS_INFO("Current robot position: %f, %f, %f,  yaw: %f", robot_position.x, robot_position.y, robot_position.z, yaw);
                ROS_INFO("Current left-right wheel angular velocities: %f, %f", left_wheel_velocity, right_wheel_velocity);
                ROS_INFO("New velocity targets:  %f, %f", command_vel.linear.x, command_vel.angular.z);
                ROS_INFO("Calculated position change: %f, %f phi change %f", pos(0), pos(1), pos(2));
                ROS_INFO("Calculated position x: %f, y: %f, heading:%f\n", last_pos(0), last_pos(1), last_pos(2));

        }

        command_velocity_publisher.publish(command_vel); // publishes the velocity command

        counter++;
        ros::spinOnce();
        control_rate.sleep();
    }


    return 0;
}
