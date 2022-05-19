#include <rl/mdl/Kinematic.h>

class Arm {
    public:
    Arm(std::string path);

    void* get_pointer2arm();

    private:
    rl::mdl::Kinematic* kin_pointer;
    rl::mdl::Kinematic kin_model;
    void* void_pointer;
 };
