#include <iostream>
#include <typeinfo>

#include <broccoli/control/Signal.hpp>
#include <broccoli/control/kinematics/AutomaticSupervisoryControl.hpp>
#include <broccoli/control/kinematics/ComfortPoseGradient.hpp>

#include <rl/math/Transform.h>
#include <rl/math/Unit.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/Model.h>
#include <rl/mdl/UrdfFactory.h>
#include <rl/mdl/JacobianInverseKinematics.h>
#include <rl/mdl/NloptInverseKinematics.h>



int main(int, char**) {

	rl::mdl::UrdfFactory factory;
	std::shared_ptr<rl::mdl::Model> model_r(factory.create("/home/joschua/Coding/invKinematics4/master-project/c++/src/urdf/yumi_right.urdf"));
	std::shared_ptr<rl::mdl::Model> model_l(factory.create("/home/joschua/Coding/invKinematics4/master-project/c++/src/urdf/yumi_left.urdf"));
	rl::mdl::Kinematic* kinematic_r = dynamic_cast<rl::mdl::Kinematic*>(model_r.get());
	rl::mdl::Kinematic* kinematic_l = dynamic_cast<rl::mdl::Kinematic*>(model_l.get());
	rl::math::Vector q_r(7);
	rl::math::Vector q_l(7);
	q_r << 10, 10, 10, 10, 10, 10, 10;
	q_l << -10, 0, 10, 0, -20, 0, 0;
	q_r *= rl::math::DEG2RAD;
	q_l *= rl::math::DEG2RAD;

	// forward kinematics for the right arm
	kinematic_r->setPosition(q_r);
	kinematic_r->forwardPosition();
	kinematic_r->calculateJacobian();

	Eigen::Matrix<double, 6, 7> myJacobian_r;

	// copy entries from RL jacobian to Eigen::jacobian in order to use it in brocolli function
	for (int i = 0; i < 7; i++){
		for (int j = 0; j < 6; j++){
			myJacobian_r(j, i) = kinematic_r->getJacobian()(j, i);
		}
	}

	
	// check if matrices are the same
	std::cout << "RLJacobian \n" << kinematic_r->getJacobian() << std::endl;
	std::cout << "myJacobian \n" << myJacobian_r << std::endl;
	std::cout << "Manipulability meassure \n" << kinematic_r->calculateManipulabilityMeasure() << std::endl;
	

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
	
	/*
	// extract orientation and position for the left arm
	rl::math::Transform t_l = kinematic_l->getOperationalPosition(0);
	rl::math::Vector3 position_l = t_l.translation();
	rl::math::Vector3 orientation_l = t_l.rotation().eulerAngles(2, 1, 0).reverse();
	std::cout << "Left Arm: Joint configuration in degrees: " << q_l.transpose() * rl::math::RAD2DEG << std::endl;
	std::cout << "Left Arm: End-effector position: [m] " << position_l.transpose() << " orientation [deg] " << orientation_l.transpose() * rl::math::RAD2DEG << std::endl;
	*/

	// do the inverse kinematics
	// instantiate vars vor ASC
	Eigen::Matrix<double, 7, 1> weightingFactors;
	Eigen::Matrix<double, 7, 1> nullSpaceGradient;
	Eigen::Matrix<double, 6, 1> actualPosition;
	Eigen::Matrix<double, 6, 1> desPosition;
	Eigen::Matrix<double, 6, 1> dPosition;
	Eigen::Matrix<double, 7, 1> jointVelocity;
	Eigen::Matrix<double, 6, 1> desVelocity;

	// instantiate vars for cpg
	Eigen::Matrix< double, 7, 1> q_min;
	Eigen::Matrix< double, 7, 1> q_max;
	//Eigen::Matrix< double, 7, 1> q_pose;

	// does not to be modified?
	weightingFactors << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

	// compute the gradient, use comfortpose class from broccoli, maybe extend class to get second part of cost function: manipulability
	// if gradient is zero then the ASC is just a resolved motion ik method
	nullSpaceGradient << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

	// obtained from forward kinematics, later the current configuration read from egm interface
	//actualPosition << 0.564465, 0.00153031,   0.418386, 52.44*rl::math::DEG2RAD, -151.976*rl::math::DEG2RAD,  157.197*rl::math::DEG2RAD;
	actualPosition << position_r.transpose()(0), position_r.transpose()(1), position_r.transpose()(2), orientation_r.transpose()(0), orientation_r.transpose()(1), orientation_r.transpose()(2);
	// add 1 mm to the position part
	dPosition << 0.01, 0.01, 0.01, 0.0, 0.0, 0.0;
	desPosition = actualPosition + dPosition;
	std::cout << "desPosition \n" << desPosition << std::endl;
	// apply vel that results from actualPosition and desPosition
	desVelocity << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

	jointVelocity << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	const double activationFactor = 1.0;
	const double dt = 0.005;

	
	// max and min joint values
	q_min << -168.5, -143.5, -168.5, -123.5, -290, -88, -229;
	q_min *= rl::math::DEG2RAD;
	q_max << 168.5, 43.5, 168.5, 80, 290, 138, 229;
	q_max *= rl::math::DEG2RAD;
	// q_pose << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	


	
	// create CompfortPoseGradient object
	broccoli::control::ComfortPoseGradient<7> cpg;
	cpg.useRangeBasedScaling(q_min, q_max);
	//cpg.setPose(q_pose);
	nullSpaceGradient = cpg.process(q_r, jointVelocity);
	std::cout << "gradient \n" << nullSpaceGradient << std::endl;
	

	// create ASC object and execute its functions for inverse kinematics
	broccoli::control::AutomaticSupervisoryControl<6,7> ik;
	ik.setWeighingMatrix(weightingFactors);
	ik.setTaskSpaceConstraintFactor(activationFactor);
	// set feedback gain for effective desired velocity
	ik.setDriftCompensationGain(1.0);
	// set the target position and velocity
	ik.setTarget(desPosition, desVelocity); // needs to be specified in order to give reasonable results for ik.outoutVelocity

	// compute the desired velocities in taskspace
	ik.process(myJacobian_r, actualPosition, nullSpaceGradient, dt);

	// obtain the computed velocity in task space
	//std::cout << "output ik \n" << ik.outputVelocity() << std::endl;
	//std::cout << "test \n" << ik.outputVelocity().value() << std::endl;
	
	// perform integration over one timestep to obtain positions that can be send to robot
	Eigen::Matrix<double, 7, 1> dq_r;
	dq_r << ik.outputVelocity().value();
	//std::cout << "test2 \n" << dq_r << std::endl;
	dq_r *= dt;

	std::cout << "current qs in RAD \n" << q_r* rl::math::RAD2DEG << std::endl;
	std::cout << "next qs in RAD \n" << (q_r+dq_r)* rl::math::RAD2DEG << std::endl;

	// copy of code above to check for correctness
	// forward kinematics for the right arm
	kinematic_r->setPosition(q_r+dq_r);
	kinematic_r->forwardPosition();

	rl::math::Transform dt_r = kinematic_r->getOperationalPosition(0);
	rl::math::Vector3 dposition_r = dt_r.translation();
	rl::math::Vector3 dorientation_r = dt_r.rotation().eulerAngles(2, 1, 0).reverse();
	std::cout << "Right Arm: Joint configuration in degrees: " << (q_r+dq_r).transpose() * rl::math::RAD2DEG << std::endl;
	std::cout << "Right Arm: End-effector position: [m] " << dposition_r.transpose() << " orientation [deg] " << dorientation_r.transpose() * rl::math::RAD2DEG << std::endl;
	
	

	return 0;
}




/* Tasks to be performed within C++
    1. read operational pose from function
        - coordinates should be stored according to how broccoli functions need it to be
    2. implement computation of Jacobian
        - search for existing solutions
    3. use broccoli to compute inverse kinematics
*/


// use broccoli to compute the joint angle via inverse kinematics


// thing about what is input and output of c++ function
