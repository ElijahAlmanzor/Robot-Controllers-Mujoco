/* This is the script that contains declarations of franka simulation initialisation information */

#pragma once

#include <string>
#include <vector>
#include <Eigen/Dense>



// Declaraton of useful constants
inline constexpr int ARM_DOF = 7;
inline constexpr double PI = 3.14159265358979323846;

// PID control constants
extern const double PID_POS_KP;
extern const double PID_POS_KI;
extern const double PID_POS_KD;

extern const double PID_ORI_KP;
extern const double PID_ORI_KI;
extern const double PID_ORI_KD;

extern const double PID_JOINT_KP;
extern const double PID_JOINT_KI;
extern const double PID_JOINT_KD;

extern const double PID_JOINT_I_LIMIT;

//USEFUL VARIABLES
extern const std::vector<std::string> FRANKA_ARM_JOINT_NAMES;

// name of the end-effector in the panda xml
extern const std::string EE_NAME;  // end-effector body from XML

// ---------------------------------
// Control targets
// ---------------------------------

// Iniital joint configuration
extern const Eigen::Matrix<double, ARM_DOF, 1> Q_ARM_INIT;

// Target pose
extern const Eigen::Isometry3d T_TARGET;
