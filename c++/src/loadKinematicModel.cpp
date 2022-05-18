#include <iostream>
#include <rl/mdl/Model.h>
#include <rl/mdl/UrdfFactory.h>
#include <rl/mdl/Kinematic.h>


void * loadKinematicModel(std::string path) {
	rl::mdl::UrdfFactory factory;

    // demo code but should be the same: https://github.com/roboticslibrary/rl/blob/master/demos/rlJacobianDemo/rlJacobianDemo.cpp
	//std::shared_ptr<rl::mdl::Kinematic> kinematic;
    //kinematic = std::dynamic_pointer_cast<rl::mdl::Kinematic>(factory.create(path));
    
    std::shared_ptr<rl::mdl::Model> model(factory.create(path));
    
    rl::mdl::Kinematic* my_p= dynamic_cast<rl::mdl::Kinematic*>(model.get());
    
    static rl::mdl::Kinematic model_kin = *(my_p);
    rl::mdl::Kinematic* my_pointer = &model_kin;
    std::cout << "pointer in func "<< my_pointer << std::endl;

    /*
    Eigen::Matrix<double, 7, 1> joint_anlges;
	joint_anlges << 2.0, 1.0, 0.0, 2.0, 1.0, 0.0, 2.0;
	// forward kinematics for the right arm
	my_pointer->setPosition(joint_anlges);
	my_pointer->forwardPosition();
	my_pointer->calculateJacobian();
	std::cout << my_pointer->getJacobian() << std::endl;
    */

	return (void*) my_pointer;
    //return kinematic;
}
