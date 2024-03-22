#ifndef SRC_TO_EULER_H
#define SRC_TO_EULER_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>


geometry_msgs::Point ToEulerAngle(geometry_msgs::Quaternion q);


#endif //SRC_TO_EULER_H
