#include <iostream>
#include <rl/mdl/Model.h>
#include <rl/mdl/UrdfFactory.h>
#include <rl/mdl/Kinematic.h>
#include "loadKinematicModel.hpp"


Arm::Arm(std::string path){
    rl::mdl::UrdfFactory factory;
    std::shared_ptr<rl::mdl::Model> model_tmp(factory.create(path));
    model = model_tmp;
    
    
    //std::cout << "adress of pointer load " << void_pointer << std::endl;
    }

std::shared_ptr<rl::mdl::Model> Arm::get_pointer2arm() {return model;}


/* class Arm {
    public:
    // constrcutor
    Arm(std::string path){
    rl::mdl::UrdfFactory factory;
    std::shared_ptr<rl::mdl::Model> model(factory.create(path));
    kin_pointer = dynamic_cast<rl::mdl::Kinematic*>(model.get());
    kin_model = *(kin_pointer);
    rl::mdl::Kinematic* tmp_pointer = &kin_model;
    void_pointer = (void*) tmp_pointer;
    }

    void* get_pointer2arm() { return  void_pointer; }

    private:
    rl::mdl::Kinematic* kin_pointer;
    rl::mdl::Kinematic kin_model;
    void* void_pointer;
 }; */
