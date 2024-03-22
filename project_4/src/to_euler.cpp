#include "../include/to_euler.h"


geometry_msgs::Point ToEulerAngle(geometry_msgs::Quaternion q) {  //JPL Quaternion to RPY
    geometry_msgs::Point euler_angles;
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    euler_angles.x = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (std::fabs(sinp) >= 1)
        euler_angles.y = std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
    else
        euler_angles.y = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    euler_angles.z = std::atan2(siny_cosp, cosy_cosp);

    return euler_angles;
}