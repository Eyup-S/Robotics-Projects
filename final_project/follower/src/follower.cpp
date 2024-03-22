/*
Date: 13.01.2024
Developed by: Eyüp Şahin
Project: Follow the target on real turtlebot
Summary: This code is developed for following the target on real turtlebot. Real Time code does not implement Dynamic Window Approach
*/ 
#include "dwa_planner.h"

/*
Date: 13.01.2024
Developed by: Eyüp Şahin
Summary: Constructor of DWAPlanner class
Input: void
Output: void
Additional info: initialization of necessary variables, subscribers and publishers 
*/ 

DWAPlanner::DWAPlanner(): control_rate(RATE)
{
    target_position_sub = nh_.subscribe("/target_position", 10, &DWAPlanner::targetPositionCallback, this);
    obstacle_position_sub = nh_.subscribe("/obstacle_position", 10, &DWAPlanner::obstaclePositionCallback, this);
    vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    curTheta = 0;
    dt = 1 / RATE; 
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;

}

/*
Date: 13.01.2024
Developed by: Eyüp Şahin
Summary: Destructor of DWAPlanner class
Input: void
Output: void
*/
DWAPlanner::~DWAPlanner()
{
    ROS_INFO("DWAPlanner DESTROYED");
}

/*
Date: 13.01.2024
Developed by: Eyüp Şahin
Summary: callback function for target position
Input: msg: target position
Output: void
*/
void DWAPlanner::targetPositionCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    t_posX = msg->x;
    t_posY = msg->y;
    target_found = true;
}

/*
Date: 13.01.2024
Developed by: Eyüp Şahin
Summary: callback function for obstacle position
Input: msg: obstacle position
Output: void
*/
void DWAPlanner::obstaclePositionCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    o_posX = msg->x;
    o_posY = msg->y;
}

/*
Date: 13.01.2024
Developed by: Eyüp Şahin
Summary: plan the trajectory of robot
Input: void
Output: void
Additional info: This function is the main function of DWAPlanner class. It calculates the velocity of robot and publishes it.
                 First takes position of target and then calculates distance and angle of target.
*/
void DWAPlanner::plan()
{
    if(!std::isnan(t_posX) && !std::isnan(t_posY)){
    //calculate the angle of target
    double theta = - atan2(t_posY, t_posX);
    //calculate the distance of target
    double dist = sqrt(pow(t_posX, 2) + pow(t_posY, 2)) - THRESHOLD_RADIUS;
    double vec_x = dist * cos(theta);

    vel_msg.angular.z =  theta - curTheta;

    //check if the angle is bigger than max angular velocity
    if (vel_msg.angular.z > MAX_VEL_THETA)
        vel_msg.angular.z = MAX_VEL_THETA;
    else if (vel_msg.angular.z < -MAX_VEL_THETA)
        vel_msg.angular.z = -MAX_VEL_THETA;
    
    //stop the robot if the angular velocity is high
    if(vel_msg.angular.z > 0.3){
        vel_msg.linear.x = 0;
    }
    else if(vel_msg.angular.z < -0.3){
        vel_msg.linear.x = 0;
    }
    else{ //if the angular velocity is low, move the robot
        vel_msg.linear.x = vec_x;

        if(vel_msg.linear.x > MAX_VEL_X)
            vel_msg.linear.x = MAX_VEL_X;
        else if(vel_msg.linear.x < -MAX_VEL_X)
            vel_msg.linear.x = -MAX_VEL_X;
    }  
    }
    //if the target is not found, stop the robot
    else 
    {
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    ROS_INFO("Target not found");
    }
}

/*
Date: 13.01.2024
Developed by: Eyüp Şahin
Summary: publish the velocity of robot
Input: void
Output: void
Additional info: Loop is created for publishing velocity of robot
*/
void DWAPlanner::run()
{
    while(ros::ok())
    {
        plan();
        ROS_INFO("vel_msg.linear.x: %f, vel_msg.angular.z: %f\n", vel_msg.linear.x, vel_msg.angular.z);
        vel_pub.publish(vel_msg);    
        ros::spinOnce();
        control_rate.sleep();
    }
}

