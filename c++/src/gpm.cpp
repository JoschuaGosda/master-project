#include <iostream>
#include <string>

#include <broccoli/control/kinematics/ComfortPoseGradient.hpp>
#include <broccoli/core/math.hpp>

#include <rl/math/Transform.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/Model.h>
#include <rl/mdl/UrdfFactory.h>


std::pair<Eigen::Matrix<double, 7, 1>, Eigen::Matrix<double, 6, 1>> gpm(Eigen::Matrix<double, 6, 1> &desPose, Eigen::Matrix<double, 6, 1> &desVelocity, 
Eigen::Matrix<double, 7, 1> &jointAngles, Eigen::Matrix<double, 7, 1> &jointVelocity, rl::mdl::Kinematic * kinematic) {


	// FORWARD KINEMATICS

	//rl::mdl::Kinematic* kinematic = kinematicPtr;

	// forward kinematics for the right arm
	kinematic->setPosition(jointAngles);
	kinematic->forwardPosition();
	kinematic->calculateJacobian();

	Eigen::Matrix<double, 6, 7> J;
	// copy entries from RL jacobian to Eigen::jacobian in order to use it in brocolli function
	for (int i = 0; i < 7; i++){
		for (int j = 0; j < 6; j++){
			J(j, i) = kinematic->getJacobian()(j, i);
		}
	}

	const clock_t t2 = clock();

	// check if matrices are the same
	//std::cout << "RLJacobian \n" << kinematic->getJacobian() << std::endl;
	//std::cout << "myJacobian \n" << J << std::endl;
	//std::cout << "Manipulability meassure \n" << kinematic->calculateManipulabilityMeasure() << std::endl;
	
	// extract orientation and position for the right arm
	rl::math::Transform t = kinematic->getOperationalPosition(0);
	rl::math::Vector3 position = t.translation();
	rl::math::Vector3 orientation = t.rotation().eulerAngles(2, 1, 0).reverse();
	std::cout << "Joint configuration in degrees: " << jointAngles.transpose() * rl::math::RAD2DEG << std::endl;
	std::cout << "FK end-effector position: [m] " << position.transpose() << " orientation [deg] " << orientation.transpose() * rl::math::RAD2DEG << std::endl;
	
	const clock_t t3 = clock(); 
	std::cout << "time for extracting & printing: \t" << t3-t2 << std::endl;
	// INVERSE KINEMATICS
	// compute translation and orientation error
	Eigen::Matrix3d desOrientation;
	// go from Euler ZYX to rotation matrix
	desOrientation = Eigen::AngleAxisd(desPose(5), Eigen::Vector3d::UnitZ())
					*Eigen::AngleAxisd(desPose(4), Eigen::Vector3d::UnitY())
					*Eigen::AngleAxisd(desPose(3), Eigen::Vector3d::UnitX());
					
	//std::cout << "reverse euler angles" << desOrientation.eulerAngles(2, 1, 0).reverse();

	//  check if these two rotation matrices match!
	//std::cout << "desOrientation \n" << desOrientation << std::endl;
	//std::cout << "RL Orientation \n" << t.rotation() << std::endl;

	//Eigen::Vector3d einheitsvektor;
	//einheitsvektor << 1.0, 0.0, 0.0;
	//std::cout << "desOrientation x einheitsvektor \n" << desOrientation* einheitsvektor << std::endl;
	//std::cout << "RL Orientation x einheitsvektor\n" << t.rotation()*einheitsvektor << std::endl;


	// define Quaternion with coefficients in the order [x, y, z, w] 
	Eigen::Vector3d desiredTranslation = desPose.head(3);
	//std::cout << "desiredTranslation" << desiredTranslation << std::endl;
	Eigen::Quaterniond desiredOrientation(desOrientation);

	// set the current positions
	Eigen::Matrix<double, 3, 1> currentTranslation;
	currentTranslation << position.x(), position.y(), position.z();
	Eigen::Quaterniond currentOrientation(t.rotation());
	
	// chose the correct quaternion such that the distance between desired and current
	// quaternion is the shortest
	if (desiredOrientation.dot(currentOrientation) < 0.0) {
		currentOrientation.coeffs() *= -1.0;
	}
	// calculate delta between quaternions
	Eigen::Quaterniond errorQuaternion = currentOrientation.inverse() * desiredOrientation;
	Eigen::Vector3d errorRotationInWorldFrame = currentOrientation * errorQuaternion.vec();

	// compute task space velocity with drift compensation
	const double gainDriftCompensation = 0.1;
	const double dt = 0.0125; // refers to 80 Hz
    Eigen::Matrix<double, 6, 1> effectiveTaskSpaceInput = Eigen::Matrix<double, 6, 1>::Zero();
	effectiveTaskSpaceInput.head(3) = gainDriftCompensation/dt * (desiredTranslation - currentTranslation)
										+ desVelocity.head(3);
	effectiveTaskSpaceInput.tail(3) = gainDriftCompensation/dt * errorRotationInWorldFrame + desVelocity.tail(3);
	std::cout << "effectiveTaskSpaceInput: " << effectiveTaskSpaceInput << std::endl;


	const clock_t t4 = clock();
	std::cout << "time for taskspace input: \t" << t4-t3 << std::endl;

	// COMPUTE CPG GRADIENT
	// define min and max values for the joints of Yumi
	Eigen::Matrix< double, 7, 1> q_min;
	Eigen::Matrix< double, 7, 1> q_max;
	q_min << -168.5, -143.5, -168.5, -123.5, -290, -88, -229;
	q_min *= rl::math::DEG2RAD;
	q_max << 168.5, 43.5, 168.5, 80, 290, 138, 229;
	q_max *= rl::math::DEG2RAD;
	// create CompfortPoseGradient object
	broccoli::control::ComfortPoseGradient<7> cpg;
	// compute CPG gradient
	cpg.useRangeBasedScaling(q_min, q_max);
	//cpg.setWeight() by default it is 1

	Eigen::Matrix<double, 7, 1> cpgGradient = Eigen::Matrix<double, 7, 1>::Zero();
	cpgGradient = cpg.process(jointAngles, jointVelocity);	// if gradient is zero then the ASC is just a resolved motion ik method

	// COMPUTE MANIPULABILTY GRADIENT
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

	Eigen::Matrix<double, 7, 1> manipGradient = Eigen::Matrix<double, 7, 1>::Zero();
    double cost = sqrt((J * J.transpose()).determinant());
    // Compute the manipulability gradient.
    for (int jj = 0; jj < 7; ++jj) {
        manipGradient[jj] = cost * ((J * J.transpose()).inverse() * dJ.at(jj) * J.transpose()).trace();
    }
	// add both gradients
	Eigen::Matrix<double, 7, 1> nullSpaceGradient = Eigen::Matrix<double, 7, 1>::Zero();
	nullSpaceGradient = 0*manipGradient + 0*cpgGradient;

	const clock_t t5 = clock();
	std::cout << "time for gradients: \t" << t5-t4 << std::endl;
	//std::cout << "gradient \n" << nullSpaceGradient << std::endl;


	// ASC desired effective velocity does not work -> implement myself
	Eigen::Matrix<double, 7, 7>  m_inverseWeighing = Eigen::Matrix<double, 7, 7> ::Identity();
	double m_activationFactorTaskSpace = 1.0;
	Eigen::Matrix<double, 7, 1> nullSpaceVelocity = -m_inverseWeighing * nullSpaceGradient;
	Eigen::Matrix<double, 7, 1> jointVelocities;
	jointVelocities = broccoli::core::math::solvePseudoInverseEquation(J, m_inverseWeighing, effectiveTaskSpaceInput, nullSpaceVelocity, m_activationFactorTaskSpace);
	std::cout << "resulting jointVelocities: \n" << jointVelocities << std::endl;

	// perform integration over one timestep to obtain positions that can be send to robot
	Eigen::Matrix<double, 7, 1> jointAnglesDelta;
	jointAnglesDelta << jointVelocities * dt;

	const clock_t t6 = clock();
	std::cout << "time for pseudo: \t" << t6-t5 << std::endl;

	//std::cout << "current qs in DEG \n" << jointAngles* rl::math::RAD2DEG << std::endl;
	//std::cout << "delta qs in DEG \n" << jointAnglesDelta * rl::math::RAD2DEG << std::endl;
	//std::cout << "next qs in DEG \n" << (jointAngles+jointAnglesDelta)* rl::math::RAD2DEG << std::endl;

	// forward kinematics with the new joints values from IK
	kinematic->setPosition(jointAngles+jointAnglesDelta);
	kinematic->forwardPosition();

	rl::math::Transform dest = kinematic->getOperationalPosition(0);
	rl::math::Vector3 dposition = dest.translation();
	rl::math::Vector3 dorientation = dest.rotation().eulerAngles(2, 1, 0).reverse();
	std::cout << "IK joint configuration in degrees: " << (jointAngles+jointAnglesDelta).transpose() * rl::math::RAD2DEG << std::endl;
	std::cout << "IK end-effector position: [m] " << dposition.transpose() << " orientation [deg] " << dorientation.transpose() * rl::math::RAD2DEG << std::endl;
	
	Eigen::Matrix<double, 6, 1> resPose;
	resPose << dposition.transpose()(0), dposition.transpose()(1), dposition.transpose()(2), dorientation.transpose()(0), dorientation.transpose()(1), dorientation.transpose()(2);

	// return desired joint angles for the next step and pose for computing the IK accuracy
	return std::make_pair((jointAngles+jointAnglesDelta), resPose);
}

