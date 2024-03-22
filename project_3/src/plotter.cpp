
#include "plotter.h"

namespace plt = matplotlibcpp;


std::vector<double> Plotter::readData(const std::string& filename, int theta_index) {
    std::vector<double> thetas;
    std::ifstream file(filename);
    double value;
    int count = 0;

    while (file >> value) {
        if (count % 3 == theta_index) {
            thetas.push_back(value);
        }
        count++;
    }

    return thetas;
}

void Plotter::plot(){
    std::string file1 = "../data/inv_vel.txt";
    std::string file2 = "../data/delta_thetas.txt";

    // Reading theta1, theta2, and theta3 from each file
    std::vector<double> theta1_file1 = readData(file1, 0);
    std::vector<double> theta2_file1 = readData(file1, 1);
    std::vector<double> theta3_file1 = readData(file1, 2);

    std::vector<double> theta1_file2 = readData(file2, 0);
    std::vector<double> theta2_file2 = readData(file2, 1);
    std::vector<double> theta3_file2 = readData(file2, 2);

    // Plotting theta1 from both files
    plt::figure();
    plt::plot(theta1_file1, "r-"); 
    plt::plot(theta1_file2, "b-"); 
    plt::title("Theta1 Comparison");
    plt::legend({"File 1", "File 2"});
    plt::show();

    // Plotting theta2 from both files
    plt::figure();
    plt::plot(theta2_file1, "r-");
    plt::plot(theta2_file2, "b-");
    plt::title("Theta2 Comparison");
    plt::legend({"File 1", "File 2"});
    plt::show();

    // Plotting theta3 from both files
    plt::figure();
    plt::plot(theta3_file1, "r-");
    plt::plot(theta3_file2, "b-");
    plt::title("Theta3 Comparison");
    plt::legend({"File 1", "File 2"});
    plt::show();

}

