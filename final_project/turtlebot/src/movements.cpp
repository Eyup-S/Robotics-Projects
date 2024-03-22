#include <fstream>

#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <ros/package.h>

#include "geometric.h"


#define OBJECT_SPEED 0.2
#define PERSON_SPEED 0.15


int main(int argc, char **argv) {

    ros::init(argc, argv, "movements");
    ros::NodeHandle nh;

    ros::Publisher movements_publisher = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);

    ros::Rate control_rate(20);
    ros::Duration(5).sleep();


    std::vector<geometry_msgs::Point> object1_poses;
    std::vector<geometry_msgs::Point> object2_poses;
    std::vector<geometry_msgs::Point> person_poses;

    {
        std::ifstream task_file(ros::package::getPath("turtlebot") + "/config/object1_poses.txt");
        std::string line;
        double number;
        ROS_INFO("Opening object 1 positions,");
        while (getline(task_file, line)) {
            object1_poses.push_back(geometry_msgs::Point());
            std::istringstream sin(line);
            sin >> object1_poses[object1_poses.size() - 1].x;
            sin >> object1_poses[object1_poses.size() - 1].y;

            std::cout << object1_poses[object1_poses.size() - 1].x << " " << object1_poses[object1_poses.size() - 1].y
                      << "\n";
        }
        task_file.close();
    }
    {
        std::ifstream task_file(ros::package::getPath("turtlebot") + "/config/object2_poses.txt");
        std::string line;
        double number;
        ROS_INFO("Opening object 2 positions,");
        while (getline(task_file, line)) {
            object2_poses.push_back(geometry_msgs::Point());
            std::istringstream sin(line);
            sin >> object2_poses[object2_poses.size() - 1].x;
            sin >> object2_poses[object2_poses.size() - 1].y;

            std::cout << object2_poses[object2_poses.size() - 1].x << " " << object2_poses[object2_poses.size() - 1].y
                      << "\n";
        }
        task_file.close();
    }
    {
        std::ifstream task_file(ros::package::getPath("turtlebot") + "/config/person_poses.txt");
        std::string line;
        double number;
        ROS_INFO("Opening person positions,");
        while (getline(task_file, line)) {
            person_poses.push_back(geometry_msgs::Point());
            std::istringstream sin(line);
            sin >> person_poses[person_poses.size() - 1].x;
            sin >> person_poses[person_poses.size() - 1].y;

            std::cout << person_poses[person_poses.size() - 1].x << " " << person_poses[person_poses.size() - 1].y
                      << "\n";
        }
        task_file.close();
    }


    gazebo_msgs::ModelState person_state, object1_state, object2_state;

    person_state.model_name = "person";
    object1_state.model_name = "object1";
    object2_state.model_name = "object2";
    person_state.reference_frame = "world";
    object1_state.reference_frame = "world";
    object2_state.reference_frame = "world";


    geometry_msgs::Point current_person_pose, current_object1_pose, current_object2_pose;

    ros::Time t0 = ros::Time::now();

    while (ros::ok()) {

        ros::Time current_time = ros::Time::now();
        ros::Duration ros_duration = current_time - t0;
        double elapsed_time = ros_duration.toSec();

        {
            double current_person_time = 0;
            int current_person_point = 0;
            while (ros::ok() &&
                   geometric::Dist(person_poses[current_person_point], person_poses[current_person_point + 1]) /
                   PERSON_SPEED < elapsed_time - current_person_time) {
                current_person_time +=
                        geometric::Dist(person_poses[current_person_point], person_poses[current_person_point + 1]) /
                        PERSON_SPEED;
                if (current_person_point != person_poses.size() - 2) {
                    current_person_point++;
                } else {
                    current_person_point = 0;
                }
            }
            current_person_pose = geometric::PointPlusVector(person_poses[current_person_point],
                                                             (elapsed_time - current_person_time) * PERSON_SPEED,
                                                             geometric::Dir(person_poses[current_person_point],
                                                                            person_poses[current_person_point + 1]));
            person_state.pose.position = current_person_pose;
            movements_publisher.publish(person_state);
            ros::spinOnce();
        }

        {
            double current_object1_time = 0;
            int current_object1_point = 0;
            while (ros::ok() &&
                   geometric::Dist(object1_poses[current_object1_point], object1_poses[current_object1_point + 1]) /
                   OBJECT_SPEED < elapsed_time - current_object1_time) {
                current_object1_time += geometric::Dist(object1_poses[current_object1_point],
                                                        object1_poses[current_object1_point + 1]) / OBJECT_SPEED;
                if (current_object1_point != object1_poses.size() - 2) {
                    current_object1_point++;
                } else {
                    current_object1_point = 0;
                }
            }
            current_object1_pose = geometric::PointPlusVector(object1_poses[current_object1_point],
                                                              (elapsed_time - current_object1_time) * OBJECT_SPEED,
                                                              geometric::Dir(object1_poses[current_object1_point],
                                                                             object1_poses[current_object1_point + 1]));
            object1_state.pose.position = current_object1_pose;
            movements_publisher.publish(object1_state);
            ros::spinOnce();
        }


        {
            double current_object2_time = 0;
            int current_object2_point = 0;
            while (ros::ok() &&
                   geometric::Dist(object2_poses[current_object2_point], object2_poses[current_object2_point + 1]) /
                   OBJECT_SPEED < elapsed_time - current_object2_time) {
                current_object2_time += geometric::Dist(object2_poses[current_object2_point],
                                                        object2_poses[current_object2_point + 1]) / OBJECT_SPEED;
                if (current_object2_point != object2_poses.size() - 2) {
                    current_object2_point++;
                } else {
                    current_object2_point = 0;
                }
            }
            current_object2_pose = geometric::PointPlusVector(object2_poses[current_object2_point],
                                                              (elapsed_time - current_object2_time) * OBJECT_SPEED,
                                                              geometric::Dir(object2_poses[current_object2_point],
                                                                             object2_poses[current_object2_point + 1]));
            object2_state.pose.position = current_object2_pose;
            movements_publisher.publish(object2_state);
            ros::spinOnce();
        }

        control_rate.sleep();
    }


    return 0;
}
