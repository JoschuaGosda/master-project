#include <rl/mdl/Kinematic.h>

class Arm {
    public:
    Arm(std::string path);

    std::shared_ptr<rl::mdl::Model> get_pointer2arm();

    private:
    //rl::mdl::Kinematic kin_model;
    std::shared_ptr<rl::mdl::Model> model;
 };
