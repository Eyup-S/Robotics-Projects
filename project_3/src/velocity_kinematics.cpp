/*
Date: 30.11.2023
Developed by: Eyüp Şahin
Project: Velocity kinematics implementation for the RRR robot 
Summary: A brief summary of the code within the specific file
*/ 

#include "../include/velocity_kinematics.h"

//constructor: clean the files before running the code
VelocityKinematics::VelocityKinematics() {
    std::string package_path = ros::package::getPath("project3");
    std::string path = package_path + "/data/";
    std::ofstream inFile(path + "jacobian.txt");
    if (!inFile.is_open()) {
        std::cerr << "Unable to open file for reading: " << path << std::endl;
        return;
    }
    inFile.close();
    std::ofstream inFile2(path + "J_v.txt");
    if (!inFile2.is_open()) {
        std::cerr << "Unable to open file for reading: " << path << std::endl;
        return;
    }
    inFile2.close();
    std::ofstream inFile3(path + "J_w.txt");
    if (!inFile3.is_open()) {
        std::cerr << "Unable to open file for reading: " << path << std::endl;
        return;
    }
    inFile3.close();

    std::ofstream inFile4(path + "delta_thetas.txt");
    if (!inFile4.is_open()) {
        std::cerr << "Unable to open file for reading: " << path << std::endl;
        return;
    }
    inFile4.close();

    std::ofstream inFile5(path + "inv_vel.txt");
    if (!inFile5.is_open()) {
        std::cerr << "Unable to open file for reading: " << path << std::endl;
        return;
    }
    inFile5.close();

    thetas[0] = 0;
    thetas[1] = 0;
    thetas[2] = 0;

    
}   

//Jacobian matrix calculation
MatrixXd VelocityKinematics::Jacobian(double theta1, double theta2, double theta3) {

    MatrixXd J(6, 3);
    J(0,0) = 1/2*(cos(theta1)*sin(theta2)*sin(theta3) + cos(theta1)*cos(theta2)*sin(theta3)) + cos(theta1)*sin(theta2);
    J(0,1) = 1/2*(sin(theta1)*cos(theta2)*sin(theta3) - sin(theta1)*sin(theta2)*sin(theta3)) + sin(theta1)*cos(theta2);
    J(0,2) = 1/2*(sin(theta1)*sin(theta2)*cos(theta3) + sin(theta1)*cos(theta2)*cos(theta3));
    J(1,0) = 1/2*(-sin(theta1)*sin(theta2)*cos(theta3) + sin(theta1)*cos(theta2)*sin(theta3)) + sin(theta1)*cos(theta2);
    J(1,1) = 1/2*(cos(theta1)*cos(theta2)*cos(theta3) + cos(theta1)*sin(theta2)*sin(theta3)) + cos(theta1)*sin(theta2);
    J(1,2) = 1/2*(-cos(theta1)*sin(theta2)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3));
    J(2,0) = 0;
    J(2,1) = 1/2*(-sin(theta2)*cos(theta3) - cos(theta2)*sin(theta3)) - sin(theta2);
    J(2,2) = 1/2*(-cos(theta2)*sin(theta3) + sin(theta2)*cos(theta3));
    J(3,0) = 0;
    J(3,1) = cos(theta1);
    J(3,2) = cos(theta1);
    J(4,0) = 0;
    J(4,1) = sin(theta1);
    J(4,2) = sin(theta1);
    J(5,0) = 1;
    J(5,1) = 0;
    J(5,2) = 0;

    return J;
}
//get the angle dot for the specific time as given in the project description
Vector3d VelocityKinematics::getAngles(int t) {

    Vector3d angles(3);
    
    if(d_t < 100){
        angles[0] = 0.1 * 0.01 * t;
        angles[1] = 0.01 * 0.01 * t;
        angles[2] = 0.1 *  0.1 * t;
    }
    else if(d_t > 100 && d_t < 500){ 
        angles[0] = 0.3;
        angles[1] = 0.1;
        angles[2] = 0.3;
    }
    else if(d_t > 500 && d_t < 600){
        angles[0] = 0.3 - 0.1 * (0.01 * t - 5);
        angles[1] = 0.1 - 0.01 * (0.01 * t - 5);
        angles[2] = 0.3 - 0.1 * (0.01 * t - 5);
    }
    return angles;
}

//print the matrices
void VelocityKinematics::printMatrices() {
    if(d_t % 100 == 0){
            std::cout << "Jacobian:\n" << jacobian << std::endl;
            std::cout << "J(v):\n" << J_v << std::endl;
            std::cout << "J(w):\n" << J_w << std::endl;
            std::cout << "delta o: " << delta_o << std::endl;
            std::cout << "delta phi: " << delta_phi << std::endl;
            std::cout << "A:\n" << A << std::endl;
            std::cout << "-----------------------------------\n";
       }
}

//save the matrices to the files
void VelocityKinematics::saveMatrix(MatrixXd J, std::string filename) {
    std::string package_path = ros::package::getPath("project3");
    std::string path = package_path + "/data/" + filename;
    std::ofstream file(path, std::ios::app);
    if (file.is_open()) {
        file << J << "\n";
    } else {
        std::cerr << "Unable to open file: " << filename << std::endl;
    }
    file.close();
}

//save the vectors to the files
void VelocityKinematics::saveVector(Vector3d v, std::string filename) {
    std::string package_path = ros::package::getPath("project3");
    std::string path = package_path + "/data/" + filename;
    std::ofstream file(path, std::ios::app);
    if (file.is_open()) {
        file << v << "\n";
    } else {
        std::cerr << "Unable to open file: " << filename << std::endl;
    }
    file.close();
}

//read the matrices from the files
std::vector<MatrixXd> VelocityKinematics::readMatrixFromFile(std::string path){
    std::ifstream file(path);
    std::vector<Eigen::MatrixXd> matrices;
    std::string line;
    std::vector<double> matrixEntries;
    int numRows = 0;
    int numCols = 0;

    if(!file.is_open()){
        std::cerr << "Unable to open file: " << path << std::endl;
        return matrices;
    }

    while (getline(file, line)) {
        if (line.empty()) {
            if (!matrixEntries.empty()) {
                Eigen::Map<Eigen::MatrixXd> matrix(matrixEntries.data(), numRows, numCols);
                matrices.push_back(matrix);
                matrixEntries.clear();
                numRows = 0;
            }
        } else {
            std::istringstream iss(line);
            double val;
            int colCount = 0;
            while (iss >> val) {
                matrixEntries.push_back(val);
                colCount++;
            }
            if (numRows == 0) numCols = colCount;
            numRows++;
        }
    }

    // Add the last matrix if the file does not end with an empty line
    if (!matrixEntries.empty()) {
        Eigen::Map<Eigen::MatrixXd> matrix(matrixEntries.data(), numRows, numCols);
        matrices.push_back(matrix);
    }

    return matrices;

}

//pseudo inverse calculation : J_pinv = (J * J_T).inverse() * J
MatrixXd VelocityKinematics::PseudoInverse(MatrixXd J) {
    MatrixXd J_T = J.transpose();
    MatrixXd J_pinv = (J * J_T).inverse() * J;
    return J_pinv;
}

//inverse velocity kinematics calculation
MatrixXd VelocityKinematics::inverseVelocityKinematics(){

    MatrixXd inv_vel = PseudoInverse(jacobian) * delta_thetas;
    return inv_vel;
}

//update the kinematic map
void VelocityKinematics::updateKinematicMap() {

    A(0,0) = 1;
    A(0,1) = - delta_phi[2];
    A(0,2) = delta_phi[1];
    A(0,3) = delta_o[0];
    A(1,0) = delta_phi[2];
    A(1,1) = 1;
    A(1,2) = - delta_phi[0];
    A(1,3) = delta_o[1];
    A(2,0) = - delta_phi[1];
    A(2,1) = delta_phi[0];
    A(2,2) = 1;
    A(2,3) = delta_o[2];
    A(3,0) = 0;
    A(3,1) = 0;
    A(3,2) = 0;
    A(3,3) = 1;

}

//run the code
std::vector<double> VelocityKinematics::run(){
    d_t++; //increment the time

    //get the angles for the specific time
    delta_thetas = getAngles(d_t);
    //update the angles
    thetas[0] += delta_thetas[0] / 2 / M_PI;
    thetas[1] += delta_thetas[1] / 2 / M_PI;
    thetas[2] += delta_thetas[2] / 2 / M_PI;

    std::cout << "thetas 0: " << thetas[0] << "thetas 1: " << thetas[1] << "thetas 2: " << thetas[2] << std::endl;

    jacobian = Jacobian(thetas[0], thetas[1], thetas[2]);
    J_v = jacobian.topRows(3);
    J_w = jacobian.bottomRows(3);

    Vector3d V = J_v * delta_thetas;
    Vector3d W = J_w * delta_thetas;

    delta_o = V * 0.01 * d_t;
    delta_phi = W * 0.01 * d_t;

    // printMatrices();
    updateKinematicMap();

    MatrixXd inv_vel = inverseVelocityKinematics(); //inverse velocity kinematics calculation

    saveMatrix(jacobian, "jacobian.txt");
    saveMatrix(J_v, "J_v.txt");
    saveMatrix(J_w, "J_w.txt");
    saveVector(delta_thetas, "delta_thetas.txt");
    saveMatrix(inv_vel, "inv_vel.txt");
    return thetas;
        
}