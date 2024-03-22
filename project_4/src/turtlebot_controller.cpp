#include "turtlebot_controller.h"

/*
Date: 20.12.2023
Developed by: Eyüp Şahin
Summary: Constructor of TurtleBotController class
Input: double radius, double distance_between_wheels
Output: void
Additional info: 
*/ 
TurtleBotController::TurtleBotController(double radius, double distance_between_wheels) {

    r = radius;
    b = distance_between_wheels;
}

/*
Date: 20.12.2023
Developed by: Eyüp Şahin
Summary: Calculate position change based on wheel velocities and heading angle.
Input: double phi, double w_r, double w_l - Current heading angle, right wheel velocity, left wheel velocity
Output: Vector3d - Position change in x, y, and phi
Additional info: Performs position change calculation using wheel velocities and heading angle.
*/ 
Vector3d TurtleBotController::calcPosChange(double phi, double w_r, double w_l) {

    double phi_dot = -(w_r * r - w_l * r)/b; 
    double x_dot = (w_r * r + w_l * r)/2 * cos(phi);
    double y_dot = (w_r * r + w_l * r)/2 * sin(phi) ;
    std::cout << "x_dot: " << x_dot << " y_dot: " << y_dot << " phi_dot: " << phi_dot << "\n";
    
    Vector3d pos(DELTA_T * x_dot,DELTA_T * y_dot,DELTA_T * phi_dot);
    return pos;
}

/*
Date: 20.12.2023
Developed by: Eyüp Şahin
Summary: Read a task file and populate goal and obstacles.
Input: None
Output: void
Additional info: Reads a task file, extracts target position, obstacle positions, and other information, and creates relevant data structures.
*/ 
void TurtleBotController::readFile() {
    std::ifstream task_file(ros::package::getPath("turtlebot") + "/config/robot_task.txt");
    std::string line;
    double number;
    ROS_INFO("Opening task file:");
    std::vector<std::vector<double>> points;
    while (getline(task_file, line)) {
        //get target position, obstacles positions
        std::vector<double> point;
        std::istringstream sin(line);
        while (sin >> number){
            point.push_back(number);
            std::cout<<number<<" ";
        }
        std::cout<<"\n";
        points.push_back(point);
    }
    task_file.close();
    goal.x = points[0][0];
    goal.y = points[0][1];
    for (int i = 1; i < points.size(); i++) { // assign obstacle positions to obstacles vector
        Obstacle obstacle;
        obstacle.position.x = points[i][0];
        obstacle.position.y = points[i][1];
        obstacle.rho = points[i][2];
        obstacle.height = points[i][3];
        obstacles.push_back(obstacle);
    }
    //print obstacles
    for (const auto& obstacle : obstacles) {
        printf("Obstacle: (%f, %f, %f, %f)\n", obstacle.position.x, obstacle.position.y, obstacle.rho, obstacle.height);
    }

}

/*
Date: 20.12.2023
Developed by: Eyüp Şahin
Summary: Calculate attractive force acting on the robot based on APF method.
Input: const geometry_msgs::Point& robotPos - Current robot position, double k_att - Attraction constant
Output: geometry_msgs::Point - Attractive force vector
Additional info: Calculates the attractive force based on the robot's position and attraction constant.
*/ 
geometry_msgs::Point TurtleBotController::calcAttractiveForce(const geometry_msgs::Point& robotPos, double k_att) {
    geometry_msgs::Point force;
    force.x = -k_att * (robotPos.x - goal.x);
    force.y = -k_att * (robotPos.y - goal.y);
    force.z = 0;
    return force;
}

/*
Date: 20.12.2023
Developed by: Eyüp Şahin
Summary: Calculate artificial repulsive force acting on the robot based on APF method.
Input: const geometry_msgs::Point& robotPos - Current robot position, double k_rep - Repulsion constant
Output: geometry_msgs::Point - Repulsive force vector
Additional info: Calculates the repulsive force based on the robot's position, obstacles, and repulsion constant.
*/ 
geometry_msgs::Point TurtleBotController::calcRepulsiveForce(const geometry_msgs::Point& robotPos, double k_rep) {
    geometry_msgs::Point force;
    force.x = 0;
    force.y = 0;
    for (const auto& obstacle : obstacles) {
    double dist = std::hypot(robotPos.x - obstacle.position.x, robotPos.y - obstacle.position.y) - (ROBOT_RADIUS + obstacle.rho);       
    double strength = k_rep * (1/dist - 1/obstacle.rho) * (1/std::pow(dist, 2));
    force.x += strength * (robotPos.x - obstacle.position.x);
    force.y += strength * (robotPos.y - obstacle.position.y);
    }
    return force;
}

/*
Date: 20.12.2023
Developed by: Eyüp Şahin
Summary: Update robot velocities based on APF method
Input: geometry_msgs::Twist& cmd_vel - Commanded velocities, const geometry_msgs::Point& robotPos - Current robot position, double heading - Robot heading angle
Output: void
Additional info: Updates linear and angular velocities based on forces and heading angle while ensuring velocity limits are not exceeded.
*/ 
void TurtleBotController::updateVelocities(geometry_msgs::Twist& cmd_vel,const geometry_msgs::Point& robotPos,double heading) {
    // set linear velocity
    calcForce(robotPos);
    double linearSpeed = std::hypot(totalForce.x, totalForce.y);
    cmd_vel.linear.x = std::min(linearSpeed, MAX_LINEAR_VELOCITY); //take MAX_LINEAR_VELOCITY if linearSpeed is greater
    if(std::hypot(robotPos.x - goal.x, robotPos.y - goal.y) < P_CONTROL_THRESHOLD)
        cmd_vel.linear.x *= P_CONTROL;
    // set angular velocity

    double angle = std::atan2(totalForce.y, totalForce.x);
    cmd_vel.angular.z = (angle - heading);

    
    if (std::abs(cmd_vel.angular.z) > MAX_ANGULAR_VELOCITY) {
        cmd_vel.angular.z = (cmd_vel.angular.z / std::abs(cmd_vel.angular.z)) * MAX_ANGULAR_VELOCITY;
    }
}

/*
Date: 20.12.2023
Developed by: Eyüp Şahin
Summary: Calculate total artificial force acting on the robot.
Input: const geometry_msgs::Point& robotPos - Current robot position
Output: void
Additional info: Calculates the total force acting on the robot by summing attractive and repulsive forces.
*/ 
void TurtleBotController::calcForce(const geometry_msgs::Point& robotPos){

    geometry_msgs::Point attractiveForce = calcAttractiveForce(robotPos, k_att);
    geometry_msgs::Point repulsiveForce = calcRepulsiveForce(robotPos, k_rep);

    
    totalForce.x = attractiveForce.x + repulsiveForce.x;
    totalForce.y = attractiveForce.y + repulsiveForce.y;
}

/*
Date: 20.12.2023
Developed by: Eyüp Şahin
Summary: Check if the robot has reached the goal position.
Input: const geometry_msgs::Point& robot_position - Current robot position
Output: bool - True if the robot has reached the goal, false otherwise
Additional info: Checks if the robot's distance to the goal is within a specified threshold.
*/ 
bool TurtleBotController::isReached(const geometry_msgs::Point& robot_position) {
    double dist = std::hypot(robot_position.x - goal.x, robot_position.y - goal.y);
    if (dist < P_CONTROL_THRESHOLD)
        p_control = 0.25;
    if (dist < SENSITIVITY_RADIUS) {
        return true;
    }
    return false;
}

/*
Date: 20.12.2023
Developed by: Eyüp Şahin
Summary: Save a path to a file.
Input: std::vector<geometry_msgs::Point> path - List of points representing the path
Output: void
Additional info: Writes the path to a file for later reference or analysis.
*/ 
void TurtleBotController::savePath(std::vector<geometry_msgs::Point> path) {
    std::ofstream path_file(ros::package::getPath("turtlebot") + "/config/path.txt");
    for (const auto& point : path) {
        path_file << point.x << " " << point.y << "\n";
    }
    path_file.close();
}


