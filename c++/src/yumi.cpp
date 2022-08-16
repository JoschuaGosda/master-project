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
    m_rotationMatrix = t.rotation(); // rotation from task frame to ee frame
    m_jacobian = kinematic->getJacobian();
    m_manipulabilty = kinematic->calculateManipulabilityMeasure();
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
	Eigen::Vector3d errorRotationInTaskFrame = currentOrientation * errorQuaternion.vec();

    // rotate the profile into ee frame, then select the correct directions for position controll and then transform back to world frame
    m_effectiveTaskSpaceInput.head(3) = m_rotationMatrix.transpose() * m_modSelectVelMatrix * m_rotationMatrix*(m_driftCompGain/m_sampleTime * (m_desPosition - m_position) + m_desPositionDot) 
										 + m_forceTaskSpaceInput;
	m_effectiveTaskSpaceInput.tail(3) = m_driftCompGain/m_sampleTime * errorRotationInTaskFrame + m_desOrientationDot;

}

void Yumi::process(){

    doForwardKinematics();
    if(m_additionalManipConstraint){
        computeManipulabilityGradient();
    }
    modifySelectionMatrix();
    compForce2VelocityController();
    compTaskSpaceInput();
    compIK();
}

Eigen::Matrix<double, 7, 1> Yumi::get_newJointValues(){
    return m_jointAngles;
}

Eigen::Matrix<double, 6, 1> Yumi::get_pose(){
    doForwardKinematics();
    Eigen::Matrix<double, 6, 1> pose;
    pose << m_position, m_orientation;
    return pose;
}

void Yumi::set_force(double force){
    if (m_hybridControl){
        m_force = force;
    }
}

void Yumi::compForce2VelocityController(){
    Eigen::Vector3d velocityEE;
    // for a postive force outcome the ee of the right arm should move in postive y direction, 
    // have a look at chosen ee frame in RS
    velocityEE << 0, m_force-m_forceOP, 0;
    velocityEE *= m_kp;
    velocityEE =  (Eigen::Matrix3d::Identity() - m_modSelectVelMatrix) * velocityEE; // perform blending - transition from position control to force control
    // transform the velocities computed in the ee frame to the task space
    m_forceTaskSpaceInput = m_rotationMatrix.transpose() * velocityEE;
}

void Yumi::compIK(){
    Eigen::Matrix<double, 7, 1> jointVelocities;
    //Eigen::Matrix<double, 7, 1> nullSpaceVelocity = -m_inverseWeighing * m_nullSpaceGradient;
    Eigen::Matrix<double, 7, 1> nullSpaceVelocity = m_nullspaceWeight * m_nullSpaceGradient;

	jointVelocities = broccoli::core::math::solvePseudoInverseEquation(m_jacobian, m_inverseWeighing, m_effectiveTaskSpaceInput,
                     nullSpaceVelocity, m_activationFactorTaskSpace);

	m_jointAnglesDelta << jointVelocities * m_sampleTime;

    m_jointAngles += m_jointAnglesDelta;
}

void Yumi::set_kp(double kp){
    m_kp = kp;
}

void Yumi::set_operationPoint(double op){
    m_forceOP = op;
}

void Yumi::set_hybridControl(bool hybridControl){
    if (hybridControl != m_hybridControl){
        m_hybridControl = hybridControl;
        if (hybridControl){
            m_selectVelMatrix <<    1, 0, 0,    // y direction is force controlled in this case
                                    0, 0, 0, 
                                    0, 0, 1;
                                    
        } else { // position control
            m_selectVelMatrix = Eigen::Matrix3d::Identity();
            m_deltaTime = 0.0; // reset variable to apply blending when changing to hybrid control again
        }
    }
    m_hybridControl = hybridControl;
}

void Yumi::set_transitionTime(double transitionTime){
    m_transitionTime = transitionTime;
}

void Yumi::modifySelectionMatrix(){
    Eigen::Matrix3d blendingMatrix;
    if(m_deltaTime < m_transitionTime){
        m_deltaTime += m_sampleTime;
        blendingMatrix <<   0, 0, 0,
                            0, (1 - m_deltaTime/m_transitionTime), 0,
                            0, 0, 0; 
    }
    else {
        blendingMatrix = Eigen::Matrix3d::Zero();
    }
    m_modSelectVelMatrix = m_selectVelMatrix + blendingMatrix; 
}

double Yumi::get_manipulabilityMeasure(){
    return m_manipulabilty;
}

void Yumi::computeManipulabilityGradient(){
    // Eigen mainly supports 2D. 3D matrices are available in develop branches. We will use a container of 2D
    // matrices here.
    std::array<Eigen::Matrix<double, 6, 7>, 7> Jdq; // NOLINT
    for (auto& matrix : Jdq) {
        matrix = Eigen::Matrix<double, 6, 7>::Zero();
    }
    Eigen::Matrix<double, 3, 7> transJ = Eigen::Matrix<double, 3, 7>::Zero();
    transJ = m_jacobian.block<3, 7>(0, 0);
    Eigen::Matrix<double, 3, 7> rotJ = Eigen::Matrix<double, 3, 7>::Zero();
    rotJ = m_jacobian.block<3, 7>(3, 0);
    const int numOfJoints = 7;
    for (int jj = 0; jj < numOfJoints; ++jj) {
        for (int ii = jj; ii < numOfJoints; ++ii) {
            Jdq.at(jj).block<3, 1>(0, ii) = rotJ.col(jj).cross(transJ.col(ii));
            Jdq.at(jj).block<3, 1>(3, ii) = rotJ.col(jj).cross(rotJ.col(ii));
            if (ii != jj) {
                Jdq.at(ii).block<3, 1>(0, jj) = Jdq.at(jj).block<3, 1>(0, ii);
            }
        }
    }
    //return Jdq;

	// Current cost.
    double cost = sqrt((m_jacobian * m_jacobian.transpose()).determinant());
    // Compute the manipulability gradient.
    //Eigen::Matrix<double, 7, 1> gradient = Eigen::Matrix<double, 7, 1>::Zero();
    for (int jj = 0; jj < 7; ++jj) {
        m_nullSpaceGradient[jj] = cost * ((m_jacobian * m_jacobian.transpose()).inverse() * Jdq.at(jj) * m_jacobian.transpose()).trace();
    }
}

void Yumi::set_additionalManipConstraint(bool flag){
    m_additionalManipConstraint = flag;
}

void Yumi::set_nullspaceWeight(double nullspaceWeight){
    m_nullspaceWeight = nullspaceWeight;
}