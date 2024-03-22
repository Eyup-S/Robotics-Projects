#ifndef TURTLEBOT_CONTROLLER_H
#define TURTLEBOT_CONTROLLER_H

/********************
*                   *
*      INCLUDES     *
*                   *
*********************/
#include <fstream>

#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <ros/package.h>
#include <Eigen/Dense>

/********************
*                   *
*      MACROS       *
*                   *
*********************/
#define WHEEL_SEPERATION 0.26
#define WHEEL_DIAMETER 0.072
#define MAX_LINEAR_VELOCITY 1.0
#define MIN_LINEAR_VELOCITY -1.0
#define MAX_ANGULAR_VELOCITY 4.5
#define MIN_ANGULAR_VELOCITY -4.5
#define ATTRACTIVE_FORCE_COEF 0.5
#define REPULSIVE_FORCE_COEF 0.9
#define P_CONTROL 0.5
#define P_CONTROL_THRESHOLD 0.25
#define SENSITIVITY_RADIUS 0.1
#define CONTROL_RATE 4.0
#define DELTA_T 1/CONTROL_RATE
#define ROBOT_RADIUS 0.16

/********************
*                   *
*     NAMESPACES    *
*                   *
*********************/
using namespace Eigen;

/********************
*                   *
*      STRUCTS      *
*                   *
*********************/

struct Obstacle {
    geometry_msgs::Point position;
    double rho;                    
    double height;                 
};

/**************************
*                         *
*     CLASS DEFINITON     *
*                         *
***************************/
class TurtleBotController{
    public:
        TurtleBotController(double, double); // constructor
        Vector3d calcPosChange(double, double, double); // calculate change in position
        void readFile(); // read obstacles from file
        geometry_msgs::Point calcAttractiveForce(const geometry_msgs::Point&, double); // calculate attractive force
        geometry_msgs::Point calcRepulsiveForce(const geometry_msgs::Point&, double);  // calculate repulsive force
        void updateVelocities(geometry_msgs::Twist&,const geometry_msgs::Point&, double); // update velocities
        void calcForce(const geometry_msgs::Point&); // calculate total force acting on the robot
        bool isReached(const geometry_msgs::Point&); // check if the robot has reached the goal
        void savePath(std::vector<geometry_msgs::Point>); // save the path taken by the robot

    private:
        double r, b; // radius of the wheel, distance between the wheels
        geometry_msgs::Point goal; // goal position
        std::vector<Obstacle> obstacles; // list of obstacles
        geometry_msgs::Point totalForce; // total force acting on the robot
        double k_att = ATTRACTIVE_FORCE_COEF; // attractive force coefficient
        double k_rep = REPULSIVE_FORCE_COEF; // repulsive force coefficient
        double p_control = P_CONTROL; // p control coefficient
};


#endif