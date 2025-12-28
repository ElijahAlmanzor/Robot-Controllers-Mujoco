#include "kinematic_pid.hpp"
#include <iostream>

// Set target (cartesian target)
// Set PID Gains (low level and high level)
// set translation and orientation control
// Forward PID simulation (high level/ cartesian) - outputs qdes
// Forward PID low-level (desired angles to joint torques with its own PID) - outputs torque + grav comp


void PID_Kinematic::set_position_gains(float kp, float ki, float kd)
{
    // Function for adjusting the controller position gains
    PID_Kinematic::hl_pos_Pgain = kp;
    PID_Kinematic::hl_pos_Igain = ki;
    PID_Kinematic::hl_pos_Dgain = kd;
}


void PID_Kinematic::set_orientation_gains(float kp, float ki, float kd)
{
    // Function for adjusting the controller position gains
    PID_Kinematic::hl_ori_Pgain = kp;
    PID_Kinematic::hl_ori_Igain = ki;
    PID_Kinematic::hl_ori_Dgain = kd;
}

void PID_Kinematic::set_joint_gains(float kp, float ki, float kd)
{
    // Function for adjusting the controller position gains
    PID_Kinematic::ll_Pgain = kp;
    PID_Kinematic::ll_Igain = ki;
    PID_Kinematic::ll_Dgain = kd;
}

