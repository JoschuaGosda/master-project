#include <iostream>
#include "gpm.hpp"

#include <Eigen/Eigen>
#include <rl/mdl/Model.h>
#include <rl/mdl/UrdfFactory.h>


int main(int, char**) {

	enum yumi_arm{YUMI_LEFT, YUMI_RIGHT};

	// tuning parameters for IK
	Eigen::Matrix<double, 7, 1> weightingFactors;
	weightingFactors << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0; // no weighting
	const double activationFactor = 1.0;
	const double dt = 0.05;
	
	// Is Values
	Eigen::Matrix<double, 6, 1> actualPosition;
	Eigen::Matrix<double, 7, 1> jointAngles;
	jointAngles << 10, 50, 70, 10, 5, 10, 10;
	//jointAngles << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	jointAngles *= rl::math::DEG2RAD;
	Eigen::Matrix<double, 7, 1> jointVelocity;
	jointVelocity << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	
	// Desired Values
	Eigen::Matrix<double, 6, 1> desPosition;
	//desPosition << 0, 0, 0, 0, 0, 0;
	desPosition << 0.31, 0.27, 0.13, 0.522, 2.51, 2.94; // values slightly modified of current pose 
	Eigen::Matrix<double, 6, 1> desVelocity;
	desVelocity << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

	std::pair<Eigen::Matrix<double, 7, 1>, Eigen::Matrix<double, 6, 1>> result;
	result = gpm(desPosition, desVelocity, jointAngles, jointVelocity, weightingFactors, activationFactor, dt, YUMI_LEFT);
	
	std::cout << "desired joint values: \n" << result.first << std::endl;
	std::cout << "current pose: \n" << result.second << std::endl;
	
	return 0;
}