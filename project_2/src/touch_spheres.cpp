#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Point.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"

#include "kinematics.h"


#define JOINT1_LOWER_LIMIT -3.14159265359
#define JOINT1_HIGHER_LIMIT 3.14159265359
#define JOINT2_LOWER_LIMIT 0
#define JOINT2_HIGHER_LIMIT 6.28318530718
#define JOINT3_LOWER_LIMIT 0
#define JOINT3_HIGHER_LIMIT 6.28318530718

#define SPHERE_IN_WS 10

//get the joint states of the robot
sensor_msgs::JointState joint_states;
void JointStatesCb(const sensor_msgs::JointStateConstPtr &msg) {
    joint_states = *msg;
}

//set the model of the sphere with orange color
int sphere_number = 0;
std::string model_xml_orange = 
    "<sdf version='1.6'>"
        "<model name='sphere" + std::to_string(sphere_number) + "'>"
            "<pose>0 0 0 0 0 0</pose>"
            "<link name='link'>"
                "<gravity>false</gravity>"
                "<visual name='visual'>"
                    "<geometry>"
                        "<sphere><radius>0.1</radius></sphere>"
                    "</geometry>"
                    "<material>"
                        "<ambient>1 0.5 0 1</ambient>"  // RGBA for orange color
                        "<diffuse>1 0.5 0 1</diffuse>"
                        "<specular>1 0.5 0 1</specular>"
                    "</material>"
                "</visual>"
            "</link>"
        "</model>"
    "</sdf>";

//set the model of the sphere with green color
std::string model_xml_green = 
    "<sdf version='1.6'>"
        "<model name='sphere" + std::to_string(sphere_number) + "'>"
            "<pose>0 0 0 0 0 0</pose>"
            "<link name='link'>"
                "<gravity>false</gravity>"
                "<visual name='visual'>"
                    "<geometry>"
                        "<sphere><radius>0.1</radius></sphere>"
                    "</geometry>"
                    "<material>"
                        "<ambient>0 1 0 1</ambient>"  // RGBA for orange color
                        "<diffuse>0 1 0 1</diffuse>"
                        "<specular>0 1 0 1</specular>"
                    "</material>"
                "</visual>"
            "</link>"
        "</model>"
    "</sdf>";

//spawn sphere at tip of the robot
bool spawnSphere(ros::ServiceClient& spawn_client, double x, double y, double z ){
    gazebo_msgs::SpawnModel spawn_service;
    spawn_service.request.model_name = "sphere" + std::to_string(sphere_number);
    spawn_service.request.model_xml = model_xml_orange;
    spawn_service.request.robot_namespace = "";
    spawn_service.request.initial_pose.position.x = x; // Set the position of the cube
    spawn_service.request.initial_pose.position.y = y;
    spawn_service.request.initial_pose.position.z = z;
    spawn_service.request.reference_frame = "world";
    if (!spawn_client.call(spawn_service)) {
        ROS_ERROR("Failed to call service /gazebo/spawn_sdf_model");
        return false;
    }
    sphere_number++;
    return true;
}

//change the color of the sphere from orange to green by removing the old one and creating a new sphere
bool changeSphereColor(ros::ServiceClient& spawn_client, ros::ServiceClient& delete_client, int sphere_num, double x, double y, double z){
    gazebo_msgs::SpawnModel spawn_service;
    gazebo_msgs::DeleteModel delete_model_srv;
    delete_model_srv.request.model_name = "sphere" + std::to_string(sphere_num);

    if (!delete_client.call(delete_model_srv)) {
        ROS_ERROR("Failed to call service /gazebo/delete_model");
        return false;
    }
    //new sphere parameters 
    spawn_service.request.model_name = "sphere" + std::to_string(sphere_num);
    spawn_service.request.model_xml = model_xml_green;
    spawn_service.request.robot_namespace = "";
    spawn_service.request.initial_pose.position.x = x; // Set the position of the cube
    spawn_service.request.initial_pose.position.y = y;
    spawn_service.request.initial_pose.position.z = z;
    spawn_service.request.reference_frame = "world";
    if (!spawn_client.call(spawn_service)) {
        ROS_ERROR("Failed to call service /gazebo/spawn_sdf_model");
        return false;
    }
    sphere_number++;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "show_workspace");
    ros::NodeHandle nh;

    //publish the joint states of the robot
    ros::Publisher joint1_publisher = nh.advertise<std_msgs::Float64>("/rrrbot/joint1_controller/command", 10);
    ros::Publisher joint2_publisher = nh.advertise<std_msgs::Float64>("/rrrbot/joint2_controller/command", 10);
    ros::Publisher joint3_publisher = nh.advertise<std_msgs::Float64>("/rrrbot/joint3_controller/command", 10);

    ros::Subscriber joint_states_subscriber = nh.subscribe("/rrrbot/joint_states", 1, JointStatesCb);
    
    //set the service client for creating and deleting model
    ros::ServiceClient spawn_client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    ros::ServiceClient delete_model_client = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");

    
    ros::Rate control_rate(15);
    ros::Duration(1).sleep();

    //set the random seed
    std::srand(static_cast<unsigned int>(time(nullptr)));

    //set the initial joint values
    std_msgs::Float64 joint1_command_value, joint2_command_value, joint3_command_value;
    joint1_command_value.data = JOINT1_LOWER_LIMIT; //
    joint2_command_value.data = JOINT2_HIGHER_LIMIT;
    joint3_command_value.data = JOINT3_HIGHER_LIMIT;

    //spawn random spheres in the workspace
    MatrixXd sphere_locations(3, SPHERE_IN_WS);
    for(int i = 0; i < SPHERE_IN_WS; i++){
        //generate random points on the unit sphere
        double theta =  (((double) rand()) / RAND_MAX) * 2 * M_PI;
        double phi = (((double) rand()) / RAND_MAX) * M_PI;
        double R = fabs((((double) rand()) / RAND_MAX));
        //convert to cartesian coordinates
        double x = R * sin(phi) * cos(theta);
        double y = R * sin(phi) * sin(theta);
        double z = R * cos(phi) + 1;
        //save the sphere location
        sphere_locations(0, i) = x;
        sphere_locations(1, i) = y;
        sphere_locations(2, i) = z;
        //spawn the sphere
        spawnSphere(spawn_client, x, y, z);
    }

    //set the kinematic parameters of the robot
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

    int counter = 0;
    int sphere_counter = 0;

    while(ros::ok()){
      counter++;
      
      if (counter == 50){
        //take every sphere and go its position
        counter = 0;
        MatrixXd thetas = kinematics.myInverseKinematics(sphere_locations(0, sphere_counter), sphere_locations(1, sphere_counter), sphere_locations(2, sphere_counter));
        joint1_command_value.data = thetas(0, 0);
        joint2_command_value.data = thetas(1, 0);
        joint3_command_value.data = thetas(2, 0);
        changeSphereColor(spawn_client, delete_model_client, sphere_counter, sphere_locations(0, sphere_counter), sphere_locations(1, sphere_counter), sphere_locations(2, sphere_counter));
        sphere_counter++;
      }      
        joint1_publisher.publish(joint1_command_value);
        joint2_publisher.publish(joint2_command_value);
        joint3_publisher.publish(joint3_command_value);
        ros::spinOnce();
        control_rate.sleep();
    }
    return 0;
}
