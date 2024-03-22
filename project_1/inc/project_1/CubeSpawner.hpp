#ifndef CUBESPAWNER_HPP
#define CUBESPAWNER_HPP

#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/ModelStates.h>
#include <string>
#include <geometry_msgs/Pose.h>
#include <fstream>
#include <sstream>
#include <random>

#define CUBE_X_COORD_MIN 0.5 // X-axis minimum value for cube to be spawned
#define CUBE_X_COORD_MAX 1 // X-axis maximum value for cube to be spawned
#define CUBE_Y_COORD_MIN 0.5 // Y-axis minimum value for cube to be spawned
#define CUBE_Y_COORD_MAX 1 // Y-axis maximum value for cube to be spawned
#define CUBE_Z_COORD 0.05 // Z-axis value for cube to be spawned

class CubeSpawner 
{

    public:
        CubeSpawner(const ros::ServiceClient&, const std::string&); //constructor
        bool spawn(const std::string&); //spawn the cube
        void callback(const gazebo_msgs::ModelStates::ConstPtr& msg); //callback function for 
        geometry_msgs::Pose getPose(); //get the pose of the cube
        std::vector<double> randomPosition(); //randomly select a position for the cube

    private:
        ros::ServiceClient model_spawner_client_; //service client for spawning the cube
        std::string model_urdf_; //urdf file for the cube
        geometry_msgs::Pose pose; //pose of the cube
};
#endif
