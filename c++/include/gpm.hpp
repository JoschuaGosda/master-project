#include <Eigen/Eigen>
#include <rl/mdl/Kinematic.h>


std::pair<Eigen::Matrix<double, 7, 1>, Eigen::Matrix<double, 6, 1>> gpm(Eigen::Matrix<double, 6, 1> &desPose, Eigen::Matrix<double, 6, 1> &desVelocity, 
Eigen::Matrix<double, 7, 1> &jointAngles, Eigen::Matrix<double, 7, 1> &jointVelocity, rl::mdl::Kinematic * kinematicPtr);
