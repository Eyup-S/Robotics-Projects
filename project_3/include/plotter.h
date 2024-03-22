

#include "matplotlibcpp.h"
#include <Eigen/Dense>
#include "velocity_kinematics.h"


class Plotter{

    public:
        std::vector<double> readData(const std::string&, int);
        void plot();

    
};