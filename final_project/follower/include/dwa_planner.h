#ifndef DWA_PLANNER_REAL_H
#define DWA_PLANNER_REAL_H

/********************
*                   *
*      INCLUDES     *
*                   *
*********************/

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"

/*************************
*                        *
*       DEFINITIONS      *
*                        *
**************************/

#define MAX_VEL_X 1.0
#define MAX_VEL_THETA 1.0
#define MIN_VEL_X -1.0
#define MIN_VEL_THETA -1.0
#define RATE 3.0
#define THRESHOLD_RADIUS 0.5
#define ALPHA 0.4 // heading coefficient
#define BETA 0.2 // velocity coefficient
#define GAMMA 0.2   // clearance coefficient
#define NO_OBSTACLE 10
#define ROBOT_RADIUS 0.20
#define LINEAR_VEL_STEP 0.1
#define ANGULAR_VEL_STEP 0.1

/**************************
*                         *
*     CLASS DEFINITON     *
*                         *
***************************/

class DWAPlanner {
    public:
        DWAPlanner(); // constructor
        ~DWAPlanner(); // destructor
        void targetPositionCallback(const geometry_msgs::Point::ConstPtr& msg); // callback function for target position
        void obstaclePositionCallback(const geometry_msgs::Point::ConstPtr& msg); // callback function for obstacle position
        bool isSafe(geometry_msgs::Twist vel); // check if the velocity is safe
        double normalizeAngle(double angle); // normalize the angle
        double distance(geometry_msgs::Point pose); // calculate the distance function
        double clearance(geometry_msgs::Twist vel); // calculate the clearance function
        double heading(geometry_msgs::Twist vel); // calculate the heading fuction
        double velocity(); // calculate the velocity function
        void velocitySpace();  // calculate the velocity space
        void selectAdmissibleVel(); // select the admissible velocity
        void plan(); // plan the trajectory of robot
        void run(); // run the node


    private:
        ros::NodeHandle nh_;
        ros::Subscriber target_position_sub;
        ros::Subscriber obstacle_position_sub;
        ros::Publisher vel_pub; 
        ros::Rate control_rate;
        geometry_msgs::Twist vel_msg;
        geometry_msgs::Point target_position;
        geometry_msgs::Point obstacle_position;
        std::vector<geometry_msgs::Twist> velocity_space;
        std::vector<geometry_msgs::Twist> admissible_vel;
        bool target_found = false;
        double t_posX, t_posY;
        double o_posX, o_posY;
        double curTheta;
        double newX, newY, newTheta;
        double dt;
};

#endif