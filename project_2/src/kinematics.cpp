#include "../include/kinematics.h"


RRRKinematics::RRRKinematics() {

}

MatrixXd RRRKinematics::DHtoA(double alfa, double theta, double a, double d) {

    MatrixXd A(4, 4);

    A(0, 0) = std::cos(theta);
    A(0, 1) = -std::sin(theta) * std::cos(alfa);
    A(0, 2) = std::sin(theta) * std::sin(alfa);
    A(0, 3) = a * std::cos(theta);

    A(1, 0) = std::sin(theta);
    A(1, 1) = std::cos(theta) * std::cos(alfa);
    A(1, 2) = -std::cos(theta) * std::sin(alfa);
    A(1, 3) = a * std::sin(theta);

    A(2, 0) = 0;
    A(2, 1) = std::sin(alfa);
    A(2, 2) = std::cos(alfa);
    A(2, 3) = d;

    A(3, 0) = 0;
    A(3, 1) = 0;
    A(3, 2) = 0;
    A(3, 3) = 1;

    return A;
}

MatrixXd RRRKinematics::PseudoInverse(MatrixXd J) {

    MatrixXd invJ = J.completeOrthogonalDecomposition().pseudoInverse();

    return invJ;
}

MatrixXd RRRKinematics::Jacobian(double theta1, double theta2, double theta3) {

    MatrixXd A01 = DHtoA(alfa1, theta1, a1, d1);
    MatrixXd A12 = DHtoA(alfa2, theta2, a2, d2);
    MatrixXd A23 = DHtoA(alfa3, theta3, a3, d3);

    MatrixXd D01 = A01;
    MatrixXd D01R = D01.block<3, 3>(0, 0);
    MatrixXd D01T = D01.block<3, 1>(0, 3);

    MatrixXd D02 = A01 * A12;
    MatrixXd D02R = D02.block<3, 3>(0, 0);
    MatrixXd D02T = D02.block<3, 1>(0, 3);

    MatrixXd D03 = A01 * A12 * A23;
    MatrixXd D03R = D03.block<3, 3>(0, 0);
    MatrixXd D03T = D03.block<3, 1>(0, 3);

    Vector3d Ri(0, 0, 1);

    Vector3d vecD01R = D01R * Ri;
    Vector3d vecD02R = D02R * Ri;

    Vector3d vecD01T(Map<Vector3d>(D01T.data(), D01T.cols() * D01T.rows()));
    Vector3d vecD02T(Map<Vector3d>(D02T.data(), D01T.cols() * D02T.rows()));
    Vector3d vecD03T(Map<Vector3d>(D03T.data(), D03T.cols() * D03T.rows()));

    MatrixXd J1 = Ri.cross(vecD03T); //R00
    MatrixXd J2 = (vecD01R).cross(vecD03T - vecD01T);
    MatrixXd J3 = (vecD02R).cross(vecD03T - vecD02T);
    MatrixXd J4 = Ri;
    MatrixXd J5 = vecD01R;
    MatrixXd J6 = vecD02R;

    MatrixXd J(3, 3); // we consider only linear velocities

    J(0, 0) = J1(0, 0);
    J(1, 0) = J1(1, 0);
    J(2, 0) = J1(2, 0);

    J(0, 1) = J2(0, 0);
    J(1, 1) = J2(1, 0);
    J(2, 1) = J2(2, 0);

    J(0, 2) = J3(0, 0);
    J(1, 2) = J3(1, 0);
    J(2, 2) = J3(2, 0);

    return J;
}

MatrixXd RRRKinematics::ForwardKinematics(double theta1, double theta2, double theta3) {

    MatrixXd A01 = DHtoA(alfa1, theta1, a1, d1);
    MatrixXd A12 = DHtoA(alfa2, theta2, a2, d2);
    MatrixXd A23 = DHtoA(alfa3, theta3, a3, d3);

    MatrixXd A03 = A01 * A12 * A23;

    MatrixXd forward = A03.block<3, 1>(0, 3);

    return forward;
}

MatrixXd RRRKinematics::InverseKinematics(double x, double y, double z) {

    MatrixXd error(3, 1);
    MatrixXd Xtarget(3, 1);

    Xtarget(0, 0) = x;  //target workspace coordinates
    Xtarget(1, 0) = y;
    Xtarget(2, 0) = z;

    MatrixXd i_1_theta(3, 1);
    MatrixXd i_theta(3, 1);

    i_theta(0, 0) = M_PI / 6; //initial guesses for joint states.
    i_theta(1, 0) = M_PI / 9;
    i_theta(2, 0) = M_PI / 10;

    MatrixXd Xcurrent = ForwardKinematics(i_theta(0, 0), i_theta(1, 0), i_theta(2, 0));

    error = Xtarget - Xcurrent;

    uint32_t counter = 0;

    while ((std::abs(error(0, 0)) > 0.00001) || (std::abs(error(1, 0)) > 0.00001) || (std::abs(error(2, 0)) > 0.00001)) {

        MatrixXd invJ = PseudoInverse(Jacobian(i_theta(0, 0), i_theta(1, 0), i_theta(2, 0)));

        i_1_theta = i_theta + invJ * error;

        Xcurrent = ForwardKinematics(i_1_theta(0, 0), i_1_theta(1, 0), i_1_theta(2, 0));

        error = Xtarget - Xcurrent;
        i_theta = i_1_theta;

        counter++;
        if (counter > 1000000) {
            std::cout << "Couldn't find the solution for inverse kinematics in time, returning process values. Is target in workspace?"<<std::endl;
            break;
        }

    }

    return i_theta;
}

MatrixXd RRRKinematics::myInverseKinematics(double x, double y,double z){
    MatrixXd thetas(3, 1);
    thetas(0, 0) = -1 * atan2(x, y);
    double a = 2 * x / sin(thetas(0, 0));
    double b = 2 * z - 1;
    std::cout << "thetas 0: "<< thetas(0,0) << "a: " << a << "b: " << b << std::endl;
    thetas(1,0) = atan2(a,b) - acos((a*a + b*b - 2) / 2) / 2;
    thetas(2,0) = acos((a*a + b*b - 2) / 2);
    if(thetas(1,0) < 0) thetas(1,0) += 2 * M_PI;
    if(thetas(2,0) < 0) thetas(2,0) += 2 * M_PI;
    return thetas;
}