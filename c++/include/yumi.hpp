#include <iostream>

#include <Eigen/Eigen>

#include <broccoli/control/kinematics/ComfortPoseGradient.hpp>
#include <broccoli/core/math.hpp>

#include <rl/math/Transform.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/Model.h>
#include <rl/mdl/UrdfFactory.h>

class Yumi {
    public:

    //functions
    Yumi(std::string path);

    void set_jointValues(Eigen::Matrix<double, 7, 1> &jointAngles, Eigen::Matrix<double, 7, 1> &jointVelocity);

    void print_pose();

    private:
    // vars for rl library
    rl::mdl::Kinematic* m_kinematic;
    rl::mdl::Kinematic m_kin_model;

    // vars to store configuration
    Eigen::Matrix<double, 6, 1> m_desPose = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> m_desVelocity = Eigen::Matrix<double, 6, 1>::Zero();

    Eigen::Matrix<double, 7, 1> m_jointAngles = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> m_jointVelocity = Eigen::Matrix<double, 7, 1>::Zero();

    rl::math::Vector3 m_position = rl::math::Vector3::Zero();
	rl::math::Vector3 m_orientation = rl::math::Vector3::Zero();

    void doForwardKinematics(rl::mdl::Kinematic* kinematic);
 };