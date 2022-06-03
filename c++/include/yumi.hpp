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
    void set_force(double force);
    void set_kp(double kp);
    void set_operationPoint(double op);
    void set_hybridControl(bool hybridControl);
    void set_transitionTime(double transitionTime);

    void process();

    Eigen::Matrix<double, 7, 1> get_newJointValues();
    Eigen::Matrix<double, 6, 1> get_pose();

    // for debugging
    void print_pose();

    private:
    // vars for rl library
    std::shared_ptr<rl::mdl::Model> m_model;

    // flag for hybrid control
    bool m_hybridControl = false; 

    // vars for tuning
    double m_driftCompGain = 1.0;
    double m_sampleTime = 0.0125;
    double m_activationFactorTaskSpace = 1.0;

    // var saving force
    double m_force = 0;
    double m_forceOP = 0; // operation around operating point of 2 newton

    // gain for control
    double m_kp = 1.0;

    // time for transition between position control to force control in the wires axis
    double m_transitionTime = 0.0;
    double m_deltaTime = 0.0; 

    // vars to store configuration
    rl::math::Vector3 m_desPosition = rl::math::Vector3::Zero();
    rl::math::Vector3 m_desPositionDot = rl::math::Vector3::Zero();
	rl::math::Vector3 m_desOrientation = rl::math::Vector3::Zero();
    rl::math::Vector3 m_desOrientationDot = rl::math::Vector3::Zero();
    Eigen::Matrix<double, 3, 3> m_rotationMatrix = Eigen::Matrix<double, 3, 3>::Zero();

    Eigen::Matrix<double, 7, 1> m_jointAngles = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> m_jointVelocity = Eigen::Matrix<double, 7, 1>::Zero();

    rl::math::Vector3 m_position = rl::math::Vector3::Zero();
	rl::math::Vector3 m_orientation = rl::math::Vector3::Zero(); //Euler ZYX convention

    Eigen::Matrix<double, 6, 7> m_jacobian = Eigen::Matrix<double, 6, 7>::Zero();

    Eigen::Matrix<double, 6, 1> m_effectiveTaskSpaceInput = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 3, 1> m_forceTaskSpaceInput = Eigen::Matrix<double, 3, 1>::Zero(); // only apply translational changes

    Eigen::Matrix<double, 7, 7>  m_inverseWeighing = Eigen::Matrix<double, 7, 7>::Identity();
    Eigen::Matrix<double, 7, 1> m_nullSpaceGradient = Eigen::Matrix<double, 7, 1>::Zero();

    Eigen::Matrix<double, 7, 1> m_jointAnglesDelta;

    Eigen::Matrix3d m_selectVelMatrix = Eigen::Matrix3d::Identity();

    // private functions
    void doForwardKinematics();
    Eigen::Matrix3d euler2rotMatrix(rl::math::Vector3 orientation);
    void compTaskSpaceInput();
    void compForce2VelocityController();
    void modifySelectionMatrix();

    
 };