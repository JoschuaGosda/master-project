#include <iostream>
#include <typeinfo>
//#include "broccoli/control/kinematics/AutomaticSupervisoryControl.hpp"

#include <rl/math/Transform.h>
#include <rl/math/Unit.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/Model.h>
#include <rl/mdl/XmlFactory.h>
#include <rl/mdl/UrdfFactory.h>
#include <rl/mdl/JacobianInverseKinematics.h>
#include <rl/mdl/NloptInverseKinematics.h>

int main(int, char**) {
    std::cout << "Hello, world!\n";

	rl::mdl::UrdfFactory factory;
	std::shared_ptr<rl::mdl::Model> model_r(factory.create("/home/joschua/Coding/invKinematics4/master-project/c++/src/urdf/yumi_right.urdf"));
	std::shared_ptr<rl::mdl::Model> model_l(factory.create("/home/joschua/Coding/invKinematics4/master-project/c++/src/urdf/yumi_left.urdf"));
	rl::mdl::Kinematic* kinematic_r = dynamic_cast<rl::mdl::Kinematic*>(model_r.get());
	rl::mdl::Kinematic* kinematic_l = dynamic_cast<rl::mdl::Kinematic*>(model_l.get());
	rl::math::Vector q_r(7);
	rl::math::Vector q_l(7);
	q_r << 0, 0, 30, 40, 50, 0, 0;
	q_l << -10, 0, 10, 0, -20, 0, 0;
	// 90 degrees at forth position is an offset
	//q << -40, -90, 0, 90, 0, 0, 0;
	q_r *= rl::math::DEG2RAD;
	q_l *= rl::math::DEG2RAD;

	// forward kinematics for the right arm
	kinematic_r->setPosition(q_r);
	kinematic_r->forwardPosition();
	kinematic_r->calculateJacobian();

	// forward kinematics for the left arm
	kinematic_l->setPosition(q_l);
	kinematic_l->forwardPosition();
	kinematic_l->calculateJacobian();

	// extract orientation and position for the right arm
	rl::math::Transform t_r = kinematic_r->getOperationalPosition(0);
	rl::math::Vector3 position_r = t_r.translation();
	rl::math::Vector3 orientation_r = t_r.rotation().eulerAngles(2, 1, 0).reverse();
	std::cout << "Right Arm: Joint configuration in degrees: " << q_r.transpose() * rl::math::RAD2DEG << std::endl;
	std::cout << "Right Arm: End-effector position: [m] " << position_r.transpose() << " orientation [deg] " << orientation_r.transpose() * rl::math::RAD2DEG << std::endl;
	
	// extract orientation and position for the left arm
	rl::math::Transform t_l = kinematic_l->getOperationalPosition(0);
	rl::math::Vector3 position_l = t_l.translation();
	rl::math::Vector3 orientation_l = t_l.rotation().eulerAngles(2, 1, 0).reverse();
	std::cout << "Left Arm: Joint configuration in degrees: " << q_l.transpose() * rl::math::RAD2DEG << std::endl;
	std::cout << "Left Arm: End-effector position: [m] " << position_l.transpose() << " orientation [deg] " << orientation_l.transpose() * rl::math::RAD2DEG << std::endl;
	
	return 0;
}




/* Tasks to be performed within C++
    1. read operational pose data from csv or other file type and store it
        - coordinates should be stored according to how broccoli functions need it to be
    2. implement computation of Jacobian
        - search for existing solutions
    3. use broccoli to compute inverse kinematics
*/


// use broccoli to compute the joint angle via inverse kinematics


// thing about what is input and output of c++ function
