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
	std::shared_ptr<rl::mdl::Model> model(factory.create("/home/joschua/Coding/invKinematics4/master-project/c++/src/urdf2/yumi_right.urdf"));
	
	rl::mdl::Kinematic* kinematic = dynamic_cast<rl::mdl::Kinematic*>(model.get());
	rl::math::Vector q(7);
	q << 10, 10, -20, 30, 50, -10 ,-10;
	// 90 degrees at forth position is an offset
	//q << -40, -90, 0, 90, 0, 0, 0;
	q *= rl::math::DEG2RAD;
	kinematic->setPosition(q);
	kinematic->forwardPosition();
	kinematic->calculateJacobian();

	rl::math::Transform t = kinematic->getOperationalPosition(0);
	rl::math::Vector3 position = t.translation();
	rl::math::Vector3 orientation = t.rotation().eulerAngles(2, 1, 0).reverse();
	std::cout << "Joint configuration in degrees: " << q.transpose() * rl::math::RAD2DEG << std::endl;
	std::cout << "End-effector position: [m] " << position.transpose() << " orientation [deg] " << orientation.transpose() * rl::math::RAD2DEG << std::endl;
	
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
