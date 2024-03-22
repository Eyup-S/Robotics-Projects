#ifndef SRC_KINEMATICS_H
#define SRC_KINEMATICS_H

#include <iostream>
#include <Eigen/Dense>


using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::Vector3d;


class RRRKinematics {
public:

    double alfa1, alfa2, alfa3;
    double d1, d2, d3;
    double a1, a2, a3;

    RRRKinematics();

    MatrixXd DHtoA(double alfa, double theta, double a, double d);
    MatrixXd Jacobian(double theta1, double theta2, double theta3);
    MatrixXd PseudoInverse(MatrixXd J);
    MatrixXd ForwardKinematics(double theta1, double theta2, double theta3);
    MatrixXd InverseKinematics(double x, double y, double z);
    MatrixXd myInverseKinematics(double x, double y, double z);

};


#endif //SRC_KINEMATICS_H