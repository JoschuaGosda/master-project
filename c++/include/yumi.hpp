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
    void set_desPoseVel(Eigen::Matrix<double, 6, 1> &desPose, Eigen::Matrix<double, 6, 1> &desVelocity);
    void set_driftCompGain(double gain);
    void set_sampleTime(double sampleTime);

    void process();

    Eigen::Matrix<double, 7, 1> get_newJointValues();
    Eigen::Matrix<double, 6, 1> get_newPose();

    // for debugging
    void print_pose();

    private:
    // vars for rl library
    std::shared_ptr<rl::mdl::Model> m_model;
    //rl::mdl::Model m_modelObj;
    //rl::mdl::Kinematic* m_kinematic;
    //rl::mdl::Kinematic m_kin_model;

    // vars for tuning
    double m_driftCompGain = 1.0;
    double m_sampleTime = 0.0125;
    double m_activationFactorTaskSpace = 1.0;

    // vars to store configuration
    rl::math::Vector3 m_desPosition = rl::math::Vector3::Zero();
    rl::math::Vector3 m_desPositionDot = rl::math::Vector3::Zero();
	rl::math::Vector3 m_desOrientation = rl::math::Vector3::Zero();
    rl::math::Vector3 m_desOrientationDot = rl::math::Vector3::Zero();

    Eigen::Matrix<double, 7, 1> m_jointAngles = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> m_jointVelocity = Eigen::Matrix<double, 7, 1>::Zero();

    rl::math::Vector3 m_position = rl::math::Vector3::Zero();
	rl::math::Vector3 m_orientation = rl::math::Vector3::Zero(); //Euler ZYX convention

    Eigen::Matrix<double, 6, 7> m_jacobian = Eigen::Matrix<double, 6, 7>::Zero();

    Eigen::Matrix<double, 6, 1> m_effectiveTaskSpaceInput = Eigen::Matrix<double, 6, 1>::Zero();

    Eigen::Matrix<double, 7, 7>  m_inverseWeighing = Eigen::Matrix<double, 7, 7> ::Identity();
    Eigen::Matrix<double, 7, 1> m_nullSpaceGradient = Eigen::Matrix<double, 7, 1>::Zero();

    Eigen::Matrix<double, 7, 1> m_jointAnglesDelta;

    // private functions
    void doForwardKinematics();
    Eigen::Matrix3d euler2rotMatrix(rl::math::Vector3 orientation);
    void compTaskSpaceInput();

    
 };