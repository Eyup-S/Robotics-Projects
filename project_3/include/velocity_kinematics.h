#ifndef VELOCITY_KINEMATICS_H
#define VELOCITY_KINEMATICS_H

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <ros/package.h>

using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::Vector3d;

class VelocityKinematics {
    public:
        VelocityKinematics();
        MatrixXd Jacobian(double theta1, double theta2, double theta3);
        Vector3d getAngles(int);
        void printMatrices();
        void saveMatrix(MatrixXd, std::string);
        void saveVector(Vector3d, std::string);
        std::vector<MatrixXd> readMatrixFromFile(std::string);
        MatrixXd PseudoInverse(MatrixXd J);
        MatrixXd inverseVelocityKinematics();
        void updateKinematicMap();
        std::vector<double> run();
        int d_t = 0;

    private:
        Vector3d delta_thetas;
        std::vector<double> thetas = {0, 0, 0};
        MatrixXd A = MatrixXd(4, 4);
        MatrixXd jacobian = MatrixXd(6, 3);
        MatrixXd J_v = MatrixXd(3, 3); 
        MatrixXd J_w = MatrixXd(3, 3);
        Vector3d delta_o = Vector3d(0, 0, 0);
        Vector3d delta_phi = Vector3d(0, 0, 0);
};


#endif