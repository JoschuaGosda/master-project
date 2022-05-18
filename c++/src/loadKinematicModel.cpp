#include <rl/mdl/Model.h>
#include <rl/mdl/UrdfFactory.h>
#include <rl/mdl/Kinematic.h>


rl::mdl::Kinematic * loadKinematicModel(std::string path) {
	rl::mdl::UrdfFactory factory;

    // demo code but should be the same: https://github.com/roboticslibrary/rl/blob/master/demos/rlJacobianDemo/rlJacobianDemo.cpp
	//std::shared_ptr<rl::mdl::Kinematic> kinematic;
    //kinematic = std::dynamic_pointer_cast<rl::mdl::Kinematic>(factory.create(path));
    
    std::shared_ptr<rl::mdl::Model> model(factory.create(path));
    

	return dynamic_cast<rl::mdl::Kinematic*>(model.get());
    //return kinematic;
}
