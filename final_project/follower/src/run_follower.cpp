/*
Date: 13.01.2024
Developed by: Eyüp Şahin
Project: Follow the target on real turtlebot
Summary: This code is developed for following the target on real turtlebot. Real Time code does not implement Dynamic Window Approach
*/ 
#include "dwa_planner.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "follower");
    DWAPlanner dwa_planner;
    dwa_planner.run();

    return 0;
}