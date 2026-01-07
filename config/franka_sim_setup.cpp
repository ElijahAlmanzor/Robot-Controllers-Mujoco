/* This is the script that contains a lot of franka initialisation information */
#include "franka_sim_setup.hpp"


// FOr identifying joint indices (qpos and qvel) - a string of joint names
const std::vector<std::string> FRANKA_ARM_JOINT_NAMES = {
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7"
};

const std::string EE_NAME = "panda_hand";  // end-effector body from XML


// Iniital joint configuration
const Eigen::Matrix<double, ARM_DOF, 1> Q_ARM_INIT = (Eigen::Matrix<double, ARM_DOF, 1>() <<
        0.0,
        -PI / 4.0,
        0.0,
        -3.0 * PI / 4.0,
        0.0,
        PI / 2.0,
        PI / 4.0).finished();

const Eigen::Isometry3d T_TARGET = []()
{
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() << 0.4, 0.0, 0.4;
    T.linear() =
        Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
    return T;
}();




// ---------------------------------
// Task-space PID gains
// ---------------------------------

const double PID_POS_KP = 50.0;
const double PID_POS_KI = 3.0;
const double PID_POS_KD = 10.0;

const double PID_ORI_KP = 50.0;
const double PID_ORI_KI = 3.0;
const double PID_ORI_KD = 10.0;

// ---------------------------------
// Joint-space PID gains
// ---------------------------------

const double PID_JOINT_KP = 50.0;
const double PID_JOINT_KI = 3.0;
const double PID_JOINT_KD = 10.0;

const double PID_JOINT_I_LIMIT = 10.0;
