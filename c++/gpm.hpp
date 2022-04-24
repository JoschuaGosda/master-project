#include <Eigen/Eigen>


std::pair<Eigen::Matrix<double, 7, 1>, Eigen::Matrix<double, 6, 1>> gpm(Eigen::Matrix<double, 6, 1> &desPosition, Eigen::Matrix<double, 6, 1> &desVelocity, 
Eigen::Matrix<double, 7, 1> &jointAngles, Eigen::Matrix<double, 7, 1> &jointVelocity,
Eigen::Matrix<double, 7, 1> &weightingFactors, const double activationFactor = 1.0, const double dt = 0.005, const int arm = 0);