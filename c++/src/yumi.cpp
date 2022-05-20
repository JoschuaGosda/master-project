#include <iostream>

#include <broccoli/control/kinematics/ComfortPoseGradient.hpp>
#include <broccoli/core/math.hpp>

#include <rl/math/Transform.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/Model.h>
#include <rl/mdl/UrdfFactory.h>

#include <yumi.hpp>

// constructor
Yumi::Yumi(std::string path){
    rl::mdl::UrdfFactory factory;
    m_model = (std::shared_ptr<rl::mdl::Model>)(factory.create(path));
    //m_modelObj= *m_model;
}

void Yumi::set_jointValues(Eigen::Matrix<double, 7, 1> &jointAngles, Eigen::Matrix<double, 7, 1> &jointVelocity){
    m_jointAngles = jointAngles;
    m_jointVelocity = jointVelocity;
}

void Yumi::doForwardKinematics(){
    rl::mdl::Kinematic* kinematic = dynamic_cast<rl::mdl::Kinematic*>(m_model.get());
    kinematic->setPosition(m_jointAngles);
    kinematic->forwardPosition();
    kinematic->calculateJacobian();
    rl::math::Transform t = kinematic->getOperationalPosition(0);
	m_position = t.translation();
	m_orientation = t.rotation().eulerAngles(2, 1, 0).reverse();
    m_jacobian = kinematic->getJacobian();
}

void Yumi::print_pose(){
    doForwardKinematics();
    std::cout << "pose " << m_position << "\n" << m_orientation << std::endl;
}

void Yumi::set_driftCompGain(double gain){
    m_driftCompGain = gain;
}

void Yumi::set_sampleTime(double sampleTime){
    m_sampleTime = sampleTime;
}

void Yumi::set_desPoseVel(Eigen::Matrix<double, 6, 1> &desPose, Eigen::Matrix<double, 6, 1> &desVelocity){

    m_desPosition = desPose.head(3);
    m_desOrientation = desPose.tail(3);
    m_desPositionDot = desVelocity.head(3);
    m_desOrientationDot = desVelocity.tail(3);
}

Eigen::Matrix3d Yumi::euler2rotMatrix(rl::math::Vector3 eulerAngles){
    Eigen::Matrix3d desOrientation;
	// go from Euler ZYX to rotation matrix
	desOrientation = Eigen::AngleAxisd(eulerAngles(2), Eigen::Vector3d::UnitZ())
					*Eigen::AngleAxisd(eulerAngles(1), Eigen::Vector3d::UnitY())
					*Eigen::AngleAxisd(eulerAngles(0), Eigen::Vector3d::UnitX());
    return desOrientation;
}

void Yumi::compTaskSpaceInput(){
    Eigen::Quaterniond desiredOrientation(euler2rotMatrix(m_desOrientation));
    Eigen::Quaterniond currentOrientation(euler2rotMatrix(m_orientation));

    // chose the correct quaternion such that the distance between desired and current
	// quaternion is the shortest
	if (desiredOrientation.dot(currentOrientation) < 0.0) {
		currentOrientation.coeffs() *= -1.0;
	}
	// calculate delta between quaternions
	Eigen::Quaterniond errorQuaternion = currentOrientation.inverse() * desiredOrientation;
	Eigen::Vector3d errorRotationInWorldFrame = currentOrientation * errorQuaternion.vec();

    m_effectiveTaskSpaceInput.head(3) = m_driftCompGain/m_sampleTime * (m_desPosition - m_position)
										+ m_desPositionDot;
	m_effectiveTaskSpaceInput.tail(3) = m_driftCompGain/m_sampleTime * errorRotationInWorldFrame + m_desOrientationDot;

}

void Yumi::process(){

    doForwardKinematics();
    compTaskSpaceInput();

    Eigen::Matrix<double, 7, 1> jointVelocities;
    Eigen::Matrix<double, 7, 1> nullSpaceVelocity = -m_inverseWeighing * m_nullSpaceGradient;

	jointVelocities = broccoli::core::math::solvePseudoInverseEquation(m_jacobian, m_inverseWeighing, m_effectiveTaskSpaceInput,
                     nullSpaceVelocity, m_activationFactorTaskSpace);

	m_jointAnglesDelta << jointVelocities * m_sampleTime;

    m_jointAngles += m_jointAnglesDelta;
}

Eigen::Matrix<double, 7, 1> Yumi::get_newJointValues(){
    return m_jointAngles;
}

Eigen::Matrix<double, 6, 1> Yumi::get_newPose(){
    doForwardKinematics();
    Eigen::Matrix<double, 6, 1> pose;
    pose << m_position, m_orientation;
    return pose;
}
