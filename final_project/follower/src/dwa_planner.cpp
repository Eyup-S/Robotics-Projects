#include "dwa_planner.h"

DWAPlanner::DWAPlanner(): control_rate(RATE)
{
    target_position_sub = nh_.subscribe("/target_position", 1, &DWAPlanner::targetPositionCallback, this);
    obstacle_position_sub = nh_.subscribe("/obstacle_position", 1, &DWAPlanner::obstaclePositionCallback, this);
    vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    curTheta = 0;
    dt = 1 / RATE;
    velocitySpace();

}

DWAPlanner::~DWAPlanner()
{
    ROS_INFO("DWAPlanner DESTROYED");
}


void DWAPlanner::targetPositionCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    target_position.x = msg->x;
    target_position.y = msg->y;
    target_found = true;
}

void DWAPlanner::obstaclePositionCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    obstacle_position.x = msg->x;
    obstacle_position.y = msg->y;
}

void DWAPlanner::velocitySpace()
{   
    for(double vel_x = MIN_VEL_X; vel_x <= MAX_VEL_X; vel_x += LINEAR_VEL_STEP)
    {   
        for(double vel_theta = MIN_VEL_THETA; vel_theta <= MAX_VEL_THETA; vel_theta += ANGULAR_VEL_STEP)
        {
            geometry_msgs::Twist vel;
            vel.linear.x = vel_x;
            vel.angular.z = vel_theta;
            velocity_space.push_back(vel);
        }
    }
}

void DWAPlanner::selectAdmissibleVel()
{
    for(int i = 0; i < velocity_space.size(); i++)
    {   
        if(isSafe(velocity_space[i]))
        {
            admissible_vel.push_back(velocity_space[i]);
        }
    }
}


bool DWAPlanner::isSafe(geometry_msgs::Twist vel)
{   
    if(!std::isnan(obstacle_position.x) && !std::isnan(obstacle_position.y))
    {   
    
        double future_angle = vel.angular.z * dt;
        double future_pose_x = vel.linear.x * cos(future_angle) * dt;
        double future_pose_y = vel.linear.x * sin(future_angle) * dt;
        double dist = sqrt(pow(obstacle_position.x - future_pose_x, 2) + pow(obstacle_position.y - future_pose_y, 2));
        if(obstacle_position.x == 0 && obstacle_position.y == 0)
        {
            dist = sqrt(pow(target_position.x - future_pose_x, 2) + pow(target_position.y - future_pose_y, 2));
        }
        if(dist < ROBOT_RADIUS + THRESHOLD_RADIUS)
        {
            return false;
        }
        else{
            return true;
        }        
    }
    else{
        return true;
    }
}

double DWAPlanner::normalizeAngle(double angle)
{
    if(angle < - M_PI)
        angle += 2 * M_PI;
    else if(angle > M_PI)
        angle -= 2 * M_PI;
    
    return angle;
}

double DWAPlanner::distance(geometry_msgs::Point pose)
{
    return sqrt(pow(pose.x, 2) + pow(pose.y, 2));
}

double DWAPlanner::clearance(geometry_msgs::Twist vel)
{
    if(!std::isnan(obstacle_position.x) && !std::isnan(obstacle_position.y))
    {
        geometry_msgs::Point future_pose;
        double future_angle = vel.angular.z * dt;
        future_pose.x = vel_msg.linear.x * cos(future_angle) * dt;
        future_pose.y = vel_msg.linear.x * sin(future_angle) * dt;
        double dist = sqrt(pow(obstacle_position.x - future_pose.x, 2) + pow(obstacle_position.y - future_pose.y, 2));
        return dist;
    }
    else{
        return NO_OBSTACLE;
    }
}

double DWAPlanner::heading(geometry_msgs::Twist vel)
{   
    double target_angle = - atan2(target_position.y, target_position.x);
    double future_angle = vel.angular.z * dt;
    double difference = normalizeAngle(target_angle - future_angle);
    return 1 - difference / M_PI;
    
}

void DWAPlanner::plan()
{
    if(target_found){
        std::cout << "Target found" << std::endl;
        admissible_vel.clear();
        selectAdmissibleVel();
        double max_score = 0;
        int max_index = 0;
        for(int i = 0; i < admissible_vel.size(); i++)
        {   
            double score = ALPHA * heading(admissible_vel[i]) + BETA * clearance(admissible_vel[i]) + GAMMA * admissible_vel[i].linear.x;
            if(score > max_score)
            {   
                max_score = score;
                max_index = i;
            }
        }
        ROS_INFO("angle: %lf", atan2(target_position.y, target_position.x));
        vel_msg.linear.x = admissible_vel[max_index].linear.x;
        vel_msg.angular.z = admissible_vel[max_index].angular.z;
        curTheta += vel_msg.angular.z * dt;
        target_found = false;
    
    }    
    else{
        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
        ROS_INFO("Target not found");
    }
}

void DWAPlanner::run()
{
    while(ros::ok())
    {      
        plan();
        ROS_INFO("vel_msg.linear.x: %f, vel_msg.angular.z: %f", vel_msg.linear.x, vel_msg.angular.z );
        vel_pub.publish(vel_msg);
        
        ros::spinOnce();
        control_rate.sleep();
    }
}

