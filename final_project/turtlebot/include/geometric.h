#ifndef SRC_GEOMETRIC_H
#define SRC_GEOMETRIC_H

#include <geometry_msgs/Point.h>

namespace geometric
{

    double Dist(geometry_msgs::Point p1, geometry_msgs::Point p2);
    geometry_msgs::Point  Dir(geometry_msgs::Point source, geometry_msgs::Point destination);
    geometry_msgs::Point PointPlusVector(geometry_msgs::Point source, double length, geometry_msgs::Point direction);

}
#endif //SRC_GEOMETRIC_H
