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
    std::shared_ptr<rl::mdl::Model> model(factory.create(path));
    rl::mdl::Kinematic* kinematic = dynamic_cast<rl::mdl::Kinematic*>(model.get());
    m_kin_model = *(kinematic);
    m_kinematic = &m_kin_model;
}

void Yumi::set_jointValues(Eigen::Matrix<double, 7, 1> &jointAngles, Eigen::Matrix<double, 7, 1> &jointVelocity){
    m_jointAngles = jointAngles;
    m_jointVelocity = jointVelocity;
}

void Yumi::doForwardKinematics(rl::mdl::Kinematic* kinematic){
    kinematic->setPosition(m_jointAngles);
    kinematic->forwardPosition();
    rl::math::Transform t = kinematic->getOperationalPosition(0);
	m_position = t.translation();
	m_orientation = t.rotation().eulerAngles(2, 1, 0).reverse();
}

void Yumi::print_pose(){
    Yumi::doForwardKinematics(&m_kin_model);
    std::cout << "pose " << m_position << "\t" << m_orientation << std::endl;
}