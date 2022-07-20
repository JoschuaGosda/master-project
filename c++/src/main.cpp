#include <iostream>
#include "yumi.hpp"

#include <Eigen/Eigen>
#include <rl/math/Unit.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/Model.h>
#include <rl/mdl/UrdfFactory.h>


int main(int, char**) {

	std::string path2yumi_r = "/home/joschua/Coding/forceControl/master-project/c++/models/urdf/yumi_right.urdf";
	Yumi yumi_r(path2yumi_r);

	Eigen::Matrix<double, 7, 1> jointAngles;
	//jointAngles << 90.48, 17.87, -25.0, 48.0, -137.0, 122.0, -74.21;
	jointAngles << -110.0, 29.85, 35.92, 49.91, 117.0, 123.0, -117.0;
	//jointAngles << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	jointAngles *= rl::math::DEG2RAD;

	Eigen::Matrix<double, 7, 1> jointVelocities;
	jointVelocities << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

	yumi_r.set_jointValues(jointAngles, jointVelocities);
	Eigen::Matrix<double, 6, 1> pose = yumi_r.get_pose();
	pose.tail(3) *= rl::math::RAD2DEG;

	std::cout << "current pose is: " << pose << std::endl;


	/*
	// Desired Values
	Eigen::Matrix<double, 6, 1> desPose;
	desPose << 0.300, 0.200, 0.200, 0.0*rl::math::DEG2RAD, 0.0*rl::math::DEG2RAD, 0.0*rl::math::DEG2RAD; // obtained from RS with stated joint values
	Eigen::Matrix<double, 6, 1> desVelocity;
	desVelocity << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

	
	yumi_r.set_desPoseVel(desPose, desVelocity);
	yumi_r.process();
	Eigen::Matrix<double, 7, 1> newJointValues;
	newJointValues = yumi_r.get_newJointValues();
	std::cout << "old joint values:  " << jointAngles << std::endl;
	std::cout << "new joint values:  " << newJointValues << std::endl;
	*/



	//enum yumi_arm{YUMI_RIGHT, YUMI_LEFT};


/* 	rl::mdl::UrdfFactory factory;
	std::shared_ptr<rl::mdl::Model> model(factory.create("/home/joschua/Coding/forceControl/master-project/c++/models/urdf/yumi_left.urdf"));
	rl::mdl::Kinematic* kinematic = dynamic_cast<rl::mdl::Kinematic*>(model.get());

	// Is Values
	Eigen::Matrix<double, 6, 1> actualPosition;
	Eigen::Matrix<double, 7, 1> jointAngles;
	//jointAngles << 90.48, 17.87, -25.0, 48.0, -137.0, 122.0, -74.21; // start position left arm, angles from RS
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
	result = gpm(desPose, desVelocity, jointAngles, jointVelocity, kinematic);
	
	std::cout << "desired joint values: \n" << result.first << std::endl;
	std::cout << "current pose: \n" << result.second << std::endl; */
	
	return 0;
}
