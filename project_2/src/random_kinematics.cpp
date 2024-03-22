#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Point.h>

#include "kinematics.h"


#define JOINT1_LOWER_LIMIT -3.14159265359
#define JOINT1_HIGHER_LIMIT 3.14159265359
#define JOINT2_LOWER_LIMIT 0
#define JOINT2_HIGHER_LIMIT 6.28318530718
#define JOINT3_LOWER_LIMIT 0
#define JOINT3_HIGHER_LIMIT 6.28318530718


sensor_msgs::JointState joint_states;

void JointStatesCb(const sensor_msgs::JointStateConstPtr &msg) {
    joint_states = *msg;
}


geometry_msgs::Point tip_position;

void LinkStatesCb(const gazebo_msgs::LinkStatesConstPtr &msg) {
    for (int i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == "robot::tip") {
            tip_position = msg->pose[i].position;
        }
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "random_kinematics");
    ros::NodeHandle nh;


    ros::Publisher joint1_publisher = nh.advertise<std_msgs::Float64>("/rrrbot/joint1_controller/command", 10);
    ros::Publisher joint2_publisher = nh.advertise<std_msgs::Float64>("/rrrbot/joint2_controller/command", 10);
    ros::Publisher joint3_publisher = nh.advertise<std_msgs::Float64>("/rrrbot/joint3_controller/command", 10);

    ros::Subscriber joint_states_subscriber = nh.subscribe("/rrrbot/joint_states", 1, JointStatesCb);
    ros::Subscriber link_states_subscriber = nh.subscribe("/gazebo/link_states", 1, LinkStatesCb);

    ros::Rate control_rate(20);

    ros::Duration(1).sleep();

    std::srand(static_cast<unsigned int>(time(nullptr))); //random number seed

    std_msgs::Float64 joint1_command_value, joint2_command_value, joint3_command_value;


    RRRKinematics kinematics;
    kinematics.a1 = 0;    // change these values according to your DH parameters
    kinematics.a2 = 0.5;
    kinematics.a3 = 0.5;
    kinematics.alfa1 = M_PI / 2;
    kinematics.alfa2 = 0;
    kinematics.alfa3 = 0;
    kinematics.d1 = 1;
    kinematics.d2 = 0;
    kinematics.d3 = 0;

    int counter = 0;
    while (ros::ok()) {

        if (counter == 100) {
            counter = 0;
            joint1_command_value.data =
                    (((double) rand()) / RAND_MAX) * (JOINT1_HIGHER_LIMIT - JOINT1_LOWER_LIMIT) + JOINT1_LOWER_LIMIT;
            joint2_command_value.data =
                    (((double) rand()) / RAND_MAX) * (JOINT2_HIGHER_LIMIT - JOINT2_LOWER_LIMIT) + JOINT2_LOWER_LIMIT + M_PI / 2;
            joint3_command_value.data =
                    (((double) rand()) / RAND_MAX) * (JOINT3_HIGHER_LIMIT - JOINT3_LOWER_LIMIT) + JOINT3_LOWER_LIMIT + M_PI / 2;
            ROS_INFO("Current joint states: %f, %f, %f", joint_states.position[0], joint_states.position[1],
                     joint_states.position[2]);
            ROS_INFO("Tip position: %f, %f, %f", tip_position.x, tip_position.y, tip_position.z);

            MatrixXd calculated_tip_position = kinematics.ForwardKinematics(joint_states.position[0] + 1.57,
                                                                            joint_states.position[1] +1.57,
                                                                            joint_states.position[2]);
            ROS_INFO("Tip position calculation with kinematics: %f, %f, %f", calculated_tip_position(0, 0),
                     calculated_tip_position(1, 0), calculated_tip_position(2, 0));

            MatrixXd calculated_joint_positions = kinematics.InverseKinematics(tip_position.x,
                                                                               tip_position.y,
                                                                               tip_position.z);
            ROS_INFO("Joint states calculation with inverse kinematics: %f, %f, %f", calculated_joint_positions(0, 0),
                                                                                     calculated_joint_positions(1, 0),
                                                                                     calculated_joint_positions(2, 0));

            ROS_INFO("New joint targets:  %f, %f, %f \n", joint1_command_value.data, joint2_command_value.data,
                     joint3_command_value.data);




        }


        joint1_publisher.publish(joint1_command_value);
        joint2_publisher.publish(joint2_command_value);
        joint3_publisher.publish(joint3_command_value);

        counter++;
        ros::spinOnce();
        control_rate.sleep();
    }


    return 0;
}
