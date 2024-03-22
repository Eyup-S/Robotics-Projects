/*
Date: 18.10.2023
Developed by: Eyüp Şahin
Project: EE 451 Project 1
Summary: CubeSpawner class is used to spawn the cube in the gazebo environment. 
*/

#include "CubeSpawner.hpp"


// Constructor
CubeSpawner::CubeSpawner(const ros::ServiceClient& model_spawner_client, const std::string& model_path) : 
    model_spawner_client_(model_spawner_client) // initialize the service client
{
    // Open and read the model file
    std::ifstream cube_file(model_path);
    if (cube_file.is_open()) {
        std::stringstream stream; // create a string stream
        stream << cube_file.rdbuf(); // read the file into the stream
        model_urdf_ = stream.str(); // convert the stream into a string
        cube_file.close(); // close the file
    } else {
        ROS_ERROR("Could not load the model file."); // print an error message
    }
}

bool CubeSpawner::spawn(const std::string& model_name)
{
    // Get a random position for the cube
    std::vector<double> position = randomPosition();
    double x = position[0];
    double y = position[1];
    double z = position[2];
    // Check if the cube is in the right position
    if(x < CUBE_X_COORD_MIN || x > CUBE_X_COORD_MAX || y < CUBE_Y_COORD_MIN || y > CUBE_Y_COORD_MAX || z != CUBE_Z_COORD){
        ROS_ERROR("The cube is not in the right position.");
        return false;
    }
    // Create the service request
    gazebo_msgs::SpawnModel spawn_model;
    spawn_model.request.model_name = model_name;
    spawn_model.request.model_xml = model_urdf_;
    spawn_model.request.robot_namespace = "/PPP_Robot";
    spawn_model.request.initial_pose.position.x = x;
    spawn_model.request.initial_pose.position.y = y;
    spawn_model.request.initial_pose.position.z = z;
    spawn_model.request.reference_frame = "world";

    // Call the service to spawn the model
    if (!model_spawner_client_.call(spawn_model) || !spawn_model.response.success) {
        //if there is a cube named same do not give error
        if(spawn_model.response.status_message == "SpawnModel: Failure - entity already exists.")
            return true;
        else
        {
        ROS_ERROR("Failed to spawn the cube model. Error message: %s", spawn_model.response.status_message.c_str());
        return false;
        }
    }

    return true;
}

// Callback function for the model states
void CubeSpawner::callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    // Find the index of the cube in the message
    int index = 0;
    for (int i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == "cube") {
            index = i; // save the index
            break; // break the loop
        }
    }
    //store the position of the cube inside geometry_msgs::Pose pose
    pose.position.x = msg->pose[index].position.x;
    pose.position.y = msg->pose[index].position.y;
    pose.position.z = msg->pose[index].position.z;

}

//get the pose of the cube
geometry_msgs::Pose CubeSpawner::getPose()
{
    return pose;
}

//generate a random number between lower and upper
double getRandomNumber(double lower, double upper)
{
    std::random_device rd; // obtain a random number from random number engine
    std::mt19937 gen(rd()); // seed the generator
    std::uniform_real_distribution<> dis(lower,upper); // define the range
    return dis(gen); //return the generated number
}

//randomly select a position for the cube
std::vector<double> CubeSpawner::randomPosition()
{
    //randomly select a position for the cube within the limitations that
    //x = [0.5,1], y= [0.5,1], z = 0.05
    std::vector<double> position;
    position.push_back(getRandomNumber(CUBE_X_COORD_MIN,CUBE_X_COORD_MAX));
    position.push_back(getRandomNumber(CUBE_Y_COORD_MIN,CUBE_Y_COORD_MAX));
    position.push_back(CUBE_Z_COORD);
    
    ROS_INFO_ONCE("Cube position: %f, %f, %f", position[0], position[1],position[2]);
    return position;
}
