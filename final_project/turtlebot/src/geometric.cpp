#include "../include/geometric.h"


namespace geometric {

    double Dist(geometry_msgs::Point p1, geometry_msgs::Point p2) {

        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    }

    geometry_msgs::Point Dir(geometry_msgs::Point source, geometry_msgs::Point destination) {
        geometry_msgs::Point dir;

        dir.x = (destination.x - source.x) / Dist(source, destination);
        dir.y = (destination.y - source.y) / Dist(source, destination);
        dir.z = (destination.z - source.z) / Dist(source, destination);

        return dir;

    }

    geometry_msgs::Point PointPlusVector(geometry_msgs::Point source, double length,
                                                    geometry_msgs::Point direction) {

        geometry_msgs::Point result;

        result.x = source.x + length * direction.x;
        result.y = source.y + length * direction.y;
        result.z = source.z + length * direction.z;

        return result;
    }
}