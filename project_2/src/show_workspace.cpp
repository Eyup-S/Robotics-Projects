#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Point.h>
#include "gazebo_msgs/SpawnModel.h"

#include "kinematics.h"


#define JOINT1_LOWER_LIMIT -3.14159265359
#define JOINT1_HIGHER_LIMIT 3.14159265359
#define JOINT2_LOWER_LIMIT 0
#define JOINT2_HIGHER_LIMIT 6.28318530718
#define JOINT3_LOWER_LIMIT 0
#define JOINT3_HIGHER_LIMIT 6.28318530718

//get the joint states of the robot
sensor_msgs::JointState joint_states;
void JointStatesCb(const sensor_msgs::JointStateConstPtr &msg) {
    joint_states = *msg;
}

//set the model of the sphere
int sphere_number = 0;
std::string model_xml = 
    "<sdf version='1.6'>\
        <model name='sphere" + std::to_string(sphere_number) + "'>\
            <pose>0 0 0 0 0 0</pose>\
            <link name='link'>\
                <gravity>false</gravity>\
                <visual name='visual'>\
                    <geometry>\
                        <sphere><radius>0.05</radius></sphere>\
                    </geometry>\
                </visual>\
            </link>\
        </model>\
    </sdf>";

//spawn sphere at tip of the robot
void spawnSphere(gazebo_msgs::SpawnModel spawn_service, ros::ServiceClient& spawn_client, double x, double y, double z){
    spawn_service.request.model_name = "sphere" + std::to_string(sphere_number);
    spawn_service.request.model_xml = model_xml;
    spawn_service.request.robot_namespace = "";
    spawn_service.request.initial_pose.position.x = x; // Set the initial position of the cube
    spawn_service.request.initial_pose.position.y = y;
    spawn_service.request.initial_pose.position.z = z;
    spawn_service.request.reference_frame = "world";
    if (!spawn_client.call(spawn_service)) {
        ROS_ERROR("Failed to call service /gazebo/spawn_sdf_model");
        return;
    }
    sphere_number++;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "show_workspace");
    ros::NodeHandle nh;

    //set the publishers and subscribers
    ros::Publisher joint1_publisher = nh.advertise<std_msgs::Float64>("/rrrbot/joint1_controller/command", 10);
    ros::Publisher joint2_publisher = nh.advertise<std_msgs::Float64>("/rrrbot/joint2_controller/command", 10);
    ros::Publisher joint3_publisher = nh.advertise<std_msgs::Float64>("/rrrbot/joint3_controller/command", 10);

    ros::Subscriber joint_states_subscriber = nh.subscribe("/rrrbot/joint_states", 1, JointStatesCb);
    
    //set the service client
    ros::ServiceClient spawn_client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel spawn_service;
    
    ros::Rate control_rate(2);
    ros::Duration(1).sleep();

    //set the initial joint values
    std_msgs::Float64 joint1_command_value, joint2_command_value, joint3_command_value;
    joint1_command_value.data = JOINT1_LOWER_LIMIT;
    joint2_command_value.data = JOINT2_HIGHER_LIMIT;
    joint3_command_value.data = JOINT3_HIGHER_LIMIT;

    
    //set the kinematics parameters
    RRRKinematics kinematics;
    kinematics.a1 = 0;    
    kinematics.a2 = 0.5;
    kinematics.a3 = 0.5;
    kinematics.alfa1 = M_PI / 2;
    kinematics.alfa2 = 0;
    kinematics.alfa3 = 0;
    kinematics.d1 = 1;
    kinematics.d2 = 0;
    kinematics.d3 = 0;

    int state_counter = 0;

    //set the initial state of the robot
    while(ros::ok()){
        state_counter++;
        if (state_counter == 2)
            break;
        joint1_publisher.publish(joint1_command_value);
        joint2_publisher.publish(joint2_command_value);
        joint3_publisher.publish(joint3_command_value);
        ros::spinOnce();
        control_rate.sleep();

    }
    
    while(ros::ok()){
        //in for loop, we will iterate over all possible joint values and publish them
        for(double i = JOINT1_LOWER_LIMIT; i < JOINT1_HIGHER_LIMIT; i+=0.3*M_PI){
            for(double j = JOINT2_LOWER_LIMIT; j < JOINT2_HIGHER_LIMIT; j += 0.3*M_PI){
                for(double k = JOINT3_LOWER_LIMIT; k < JOINT3_HIGHER_LIMIT; k += 0.3*M_PI){
                    joint1_command_value.data = i;
                    joint2_command_value.data = j;
                    joint3_command_value.data = k;
                    joint1_publisher.publish(joint1_command_value);
                    joint2_publisher.publish(joint2_command_value);
                    joint3_publisher.publish(joint3_command_value);                    
                    ros::spinOnce();
                    control_rate.sleep();
                    MatrixXd pose = kinematics.ForwardKinematics(i + M_PI / 2 , j + M_PI / 2, k);
                    spawnSphere(spawn_service, spawn_client, pose(0, 0), pose(1, 0), pose(2, 0));
                }
            }
            
        }

    }


    return 0;
}