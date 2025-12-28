#include "kinematic_pid.hpp"
#include <iostream>


// 1. Set PID Gains (low level and high level) - DONE!
// 2. Set target (HTM) - DONE!
// 3. set joint_target - useful for initiating the joint position at the start - DONE
// 4. convert set targets to position, orientation targets - DONE!
// 5. Get translation, orientation, and joint error
// 5.b PID object translation, orientation and joint error
// 6. Forward PID simulation (high level - cartesian with orientation) - outputs qdes
// 7. Forward PID low-level (desired angles to joint torques with its own PID) - outputs torque + grav comp


void PID_Kinematic::set_position_gains(float kp, float ki, float kd)
{
    // Function for adjusting the controller position gains
    hl_pos_Pgain = kp;
    hl_pos_Igain = ki;
    hl_pos_Dgain = kd;
}


void PID_Kinematic::set_orientation_gains(float kp, float ki, float kd)
{
    // Function for adjusting the controller position gains
    hl_ori_Pgain = kp;
    hl_ori_Igain = ki;
    hl_ori_Dgain = kd;
}

void PID_Kinematic::set_joint_gains(float kp, float ki, float kd)
{
    // Function for adjusting the controller position gains
    ll_Pgain = kp;
    ll_Igain = ki;
    ll_Dgain = kd;
}

void PID_Kinematic::set_target_pose(const Eigen::Isometry3d& pose)
{
    this->target_pose = pose;

    Eigen::Vector3d tp = pose.translation();
    Eigen::Quaterniond tq(pose.rotation());

    set_target_position(tp);
    set_target_orientation(tq);
}

void PID_Kinematic::set_target_position(const Eigen::Vector3d& position)
{
    this->target_position = position;

    // Overwrite the position in pose just in case
    this->target_pose.translation() = position;
}

void PID_Kinematic::set_target_orientation(const Eigen::Quaterniond& orientation)
{
    this->target_orientation = orientation;

    // Overwrite the orientation in pose just in case
    this->target_pose.linear() = orientation.toRotationMatrix();

}

void PID_Kinematic::set_target_joint(const Eigen::VectorXd& joint)
{

    this->target_joint = joint;
}
