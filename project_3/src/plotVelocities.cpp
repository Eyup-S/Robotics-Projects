#include "matplotlibcpp.h"
#include <Eigen/Dense>
#include "velocity_kinematics.h"


using Eigen::MatrixXd;

int main() {

    std::vector<MatrixXd> matrices;
    VelocityKinematics velocity_kinematics;
    matrices = velocity_kinematics.readMatrixFromFile("jacobian.txt");
    // print first matrix
    
    return 0;
}