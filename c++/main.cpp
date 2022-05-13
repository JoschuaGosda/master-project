#include <iostream>
#include "gpm.hpp"

#include <Eigen/Eigen>
#include <rl/math/Unit.h>


int main(int, char**) {

	enum yumi_arm{YUMI_RIGHT, YUMI_LEFT};

	// Is Values
	Eigen::Matrix<double, 6, 1> actualPosition;
	Eigen::Matrix<double, 7, 1> jointAngles;
	//jointAngles << 90.0, 20.0, -25.0, 48.0, -137.0, 70.0, -25.0; // start position left arm, angles from RS
	jointAngles << -110.0, 29.85, 35.92, 49.91, 117.0, 123.0, -117.0; // start position right arm, angles from RS
	//jointAngles << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	jointAngles *= rl::math::DEG2RAD;
	
	Eigen::Matrix<double, 7, 1> jointVelocity;
	jointVelocity << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	
	// Desired Values
	Eigen::Matrix<double, 6, 1> desPose;
	desPose << 0.300, 0.200, 0.200, 33.4*rl::math::DEG2RAD, 157.0*rl::math::DEG2RAD, 39.4*rl::math::DEG2RAD; // obtained from RS with stated joint values
	Eigen::Matrix<double, 6, 1> desVelocity;
	desVelocity << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

	std::pair<Eigen::Matrix<double, 7, 1>, Eigen::Matrix<double, 6, 1>> result;
	result = gpm(desPose, desVelocity, jointAngles, jointVelocity, YUMI_RIGHT);
	
	std::cout << "desired joint values: \n" << result.first << std::endl;
	std::cout << "current pose: \n" << result.second << std::endl;
	
	return 0;
}