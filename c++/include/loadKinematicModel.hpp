#include <rl/mdl/Kinematic.h>

class Arm_R {
    public:
    Arm_R(std::string path);

    void* get_pointer2arm();

    private:
    rl::mdl::Kinematic* kin_pointer;
    rl::mdl::Kinematic kin_model;
    void* void_pointer;
 };

 class Arm_L {
    public:
    Arm_L(std::string path);

    void* get_pointer2arm();

    private:
    rl::mdl::Kinematic* kin_pointer;
    rl::mdl::Kinematic kin_model;
    void* void_pointer;
 };
