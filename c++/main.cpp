#include <iostream>
#include "gpm.hpp"

#include <Eigen/Eigen>
#include <rl/mdl/Model.h>
#include <rl/mdl/UrdfFactory.h>


int main(int, char**) {

	enum yumi_arm{YUMI_LEFT, YUMI_RIGHT};

	// Is Values
	Eigen::Matrix<double, 6, 1> actualPosition;
	Eigen::Matrix<double, 7, 1> jointAngles;
	// RIGHT ARM
	//jointAngles << 10, 0, -70, -60, 0, 40, 135; //test with this setting shows up to 0.8 mm deviation
	//jointAngles << 0, 0, 0, -60, 0, 0, 135; calibration with this setting
	// LEFT ARM
	//jointAngles << 60, -40, 50, 0, 140, 80, 45; //test1 - works
	//jointAngles << 30, -70, 40, 25, -100, 90, 10; // test2 - works
	//jointAngles << 0, 0, 0, -60, 0, 0, 45; //calibration with this setting
	//jointAngles << 90.48, 17.87, -25.09, 48, -137, 122, -74.21; // start position left arm, angles from RS
	jointAngles << 90.0, 20.0, -25.0, 48.0, -137.0, 70.0, -25.0; // start position left arm, angles from RS
	//jointAngles << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;


	jointAngles *= rl::math::DEG2RAD;
	Eigen::Matrix<double, 7, 1> jointVelocity;
	jointVelocity << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	
	// Desired Values
	Eigen::Matrix<double, 6, 1> desPose;
	//desPosition << 0, 0, 0, 0, 0, 0;
	//desPose << 0.300, 0.200, 0.200, -131.4*rl::math::DEG2RAD, 17.4*rl::math::DEG2RAD, -131.8*rl::math::DEG2RAD; // values slightly modified of current pose 
	desPose << 0.300, 0.200, 0.200, 33.4*rl::math::DEG2RAD, 157.0*rl::math::DEG2RAD, 39.4*rl::math::DEG2RAD; 
	Eigen::Matrix<double, 6, 1> desVelocity;
	desVelocity << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

	std::pair<Eigen::Matrix<double, 7, 1>, Eigen::Matrix<double, 6, 1>> result;
	result = gpm(desPose, desVelocity, jointAngles, jointVelocity, YUMI_RIGHT);
	
	std::cout << "desired joint values: \n" << result.first << std::endl;
	std::cout << "current pose: \n" << result.second << std::endl;
	
	return 0;
}