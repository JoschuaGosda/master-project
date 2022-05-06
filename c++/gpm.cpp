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


std::pair<Eigen::Matrix<double, 7, 1>, Eigen::Matrix<double, 6, 1>> gpm(Eigen::Matrix<double, 6, 1> &desPosition, Eigen::Matrix<double, 6, 1> &desVelocity, 
Eigen::Matrix<double, 7, 1> &jointAngles, Eigen::Matrix<double, 7, 1> &jointVelocity,
Eigen::Matrix<double, 7, 1> &weightingFactors, const double activationFactor = 1.0, const double dt = 0.005, const int arm = 0) {
/*
	desPosition and desVelocity have to be modified
*/
	// VARIABLES INITIALIZATION
	Eigen::Matrix<double, 6, 7> J;

	// instantiate vars vor Automatic Supervisory Control (ASC)
	Eigen::Matrix<double, 7, 1> nullSpaceGradient = Eigen::Matrix<double, 7, 1>::Zero();
	Eigen::Matrix<double, 7, 1> manipGradient = Eigen::Matrix<double, 7, 1>::Zero();
	Eigen::Matrix<double, 6, 1> actualPosition;

	// create ASC object and execute its functions for inverse kinematics
	broccoli::control::AutomaticSupervisoryControl<6,7> ik;
	
	// define min and max values for the joints of Yumi
	Eigen::Matrix< double, 7, 1> q_min;
	Eigen::Matrix< double, 7, 1> q_max;
	q_min << -168.5, -143.5, -168.5, -123.5, -290, -88, -229;
	q_min *= rl::math::DEG2RAD;
	q_max << 168.5, 43.5, 168.5, 80, 290, 138, 229;
	q_max *= rl::math::DEG2RAD;
	// instantiate vars for Comfort Pose Gradient (CPG)
	Eigen::Matrix<double, 7, 1> cpgGradient = Eigen::Matrix<double, 7, 1>::Zero();
	// create CompfortPoseGradient object
	broccoli::control::ComfortPoseGradient<7> cpg;
	
	// desired joint values after doing the inverse kinematics
	Eigen::Matrix<double, 7, 1> desq;
	
	// FORWARD KINEMATICS
	rl::mdl::UrdfFactory factory;
	
	// Use the left arm as default PROBLEM HERE!!!!!
	std::shared_ptr<rl::mdl::Model> model(factory.create("/home/joschua/Coding/forceControl/master-project/c++/src/urdf/yumi_right.urdf"));
	// overwrite model if right arm is choosen
	//if (arm == 1){
//		std::shared_ptr<rl::mdl::Model> model(factory.create("/home/joschua/Coding/forceControl/master-project/c++/src/urdf/yumi_right.urdf"));
//	};

	rl::mdl::Kinematic* kinematic = dynamic_cast<rl::mdl::Kinematic*>(model.get());

	// forward kinematics for the right arm
	kinematic->setPosition(jointAngles);
	kinematic->forwardPosition();
	kinematic->calculateJacobian();

	// copy entries from RL jacobian to Eigen::jacobian in order to use it in brocolli function
	for (int i = 0; i < 7; i++){
		for (int j = 0; j < 6; j++){
			J(j, i) = kinematic->getJacobian()(j, i);
		}
	}

	// check if matrices are the same
	//std::cout << "RLJacobian \n" << kinematic->getJacobian() << std::endl;
	//std::cout << "myJacobian \n" << J << std::endl;
	//std::cout << "Manipulability meassure \n" << kinematic->calculateManipulabilityMeasure() << std::endl;
	
	// extract orientation and position for the right arm
	rl::math::Transform t = kinematic->getOperationalPosition(0);
	rl::math::Vector3 position = t.translation();
	rl::math::Vector3 orientation = t.rotation().eulerAngles(2, 1, 0).reverse();
	//std::cout << "Joint configuration in degrees: " << jointAngles.transpose() * rl::math::RAD2DEG << std::endl;
	//std::cout << "End-effector position: [m] " << position.transpose() << " orientation [deg] " << orientation.transpose() * rl::math::RAD2DEG << std::endl;
	

	// INVERSE KINEMATICS
	// obtained from forward kinematics, later the current configuration read from egm interface
	actualPosition << position.transpose()(0), position.transpose()(1), position.transpose()(2), orientation.transpose()(0), orientation.transpose()(1), orientation.transpose()(2);
	// add 1 mm to the position part
	//dPosition << 0.01, 0.01, 0.01, 0.0, 0.0, 0.0;
	//desPosition = actualPosition + dPosition;
	//std::cout << "desPosition \n" << desPosition << std::endl;
	// apply vel that results from actualPosition and desPosition
	//desVelocity << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	// TODO: this needs to be changed soon!
	//desVelocity << dPosition * 1/dt; 
	
	// compute CPG gradient
	cpg.useRangeBasedScaling(q_min, q_max);
	cpgGradient = cpg.process(jointAngles, jointVelocity);	// if gradient is zero then the ASC is just a resolved motion ik method
	
	// compute Jacobian derivative - code snippet from Jonas Wittmann
	std::array<Eigen::Matrix<double, 6, 7>, 7> dJ; // NOLINT
    for (auto& matrix : dJ) {
        matrix = Eigen::Matrix<double, 6, 7>::Zero();
    }
    Eigen::Matrix<double, 3, 7> transJ = Eigen::Matrix<double, 3, 7>::Zero();
    transJ = J.block<3, 7>(0, 0);
    Eigen::Matrix<double, 3, 7> rotJ = Eigen::Matrix<double, 3, 7>::Zero();
    rotJ = J.block<3, 7>(3, 0);
    const int numOfJoints = 7;
    for (int jj = 0; jj < numOfJoints; ++jj) {
        for (int ii = jj; ii < numOfJoints; ++ii) {
            dJ.at(jj).block<3, 1>(0, ii) = rotJ.col(jj).cross(transJ.col(ii));
            dJ.at(jj).block<3, 1>(3, ii) = rotJ.col(jj).cross(rotJ.col(ii));
            if (ii != jj) {
                dJ.at(ii).block<3, 1>(0, jj) = dJ.at(jj).block<3, 1>(0, ii);
            }
        }
    }

	// compute Manipulability gradient
	// Compute the derivative of the Jacobian w.r.t. the joint angles.
    //Eigen::Matrix<double, 6, 7>, 7> ddJ_r = jacobianDerivative(J);
    // Current cost.
    double cost = sqrt((J * J.transpose()).determinant());
    // Compute the manipulability gradient.
    
    for (int jj = 0; jj < 7; ++jj) {
        manipGradient[jj] = cost * ((J * J.transpose()).inverse() * dJ.at(jj) * J.transpose()).trace();
    }

	// add both gradients
	nullSpaceGradient = manipGradient + cpgGradient;
	//std::cout << "gradient \n" << nullSpaceGradient << std::endl;

	// do the inverse kinematics
	ik.setWeighingMatrix(weightingFactors);
	ik.setTaskSpaceConstraintFactor(activationFactor);
	// set feedback gain for effective desired velocity
	ik.setDriftCompensationGain(1.0);
	// set the target position and velocity
	ik.setTarget(desPosition, desVelocity); // needs to be specified in order to give reasonable results for ik.outoutVelocity

	// compute the desired velocities in taskspace
	ik.process(J, actualPosition, nullSpaceGradient, dt);

	// obtain the computed velocity in task space
	//std::cout << "output ik \n" << ik.outputVelocity() << std::endl;
	//std::cout << "test \n" << ik.outputVelocity().value() << std::endl;
	
	// perform integration over one timestep to obtain positions that can be send to robot
	desq << ik.outputVelocity().value();
	//std::cout << "test2 \n" << dq_r << std::endl;
	desq *= dt;

	//std::cout << "current qs in DEG \n" << jointAngles* rl::math::RAD2DEG << std::endl;
	//std::cout << "next qs in DEG \n" << (jointAngles+desq)* rl::math::RAD2DEG << std::endl;

	// copy of code above to check for correctness
	// forward kinematics for the right arm
	kinematic->setPosition(jointAngles+desq);
	kinematic->forwardPosition();

	rl::math::Transform dest = kinematic->getOperationalPosition(0);
	rl::math::Vector3 dposition = dest.translation();
	rl::math::Vector3 dorientation = dest.rotation().eulerAngles(2, 1, 0).reverse();
	//std::cout << "Right Arm: Joint configuration in degrees: " << (jointAngles+desq).transpose() * rl::math::RAD2DEG << std::endl;
	//std::cout << "Right Arm: End-effector position: [m] " << dposition.transpose() << " orientation [deg] " << dorientation.transpose() * rl::math::RAD2DEG << std::endl;
	
	// return desired joint angles for the next step and current pose for computing the IK accuracy
	//return desq;
	return std::make_pair(desq, actualPosition);
}