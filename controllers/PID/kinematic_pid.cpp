#include "kinematic_pid.hpp"
#include <iostream>


// Initialise this controller

void PID_Kinematic::init(int num_joints)
{
    if (num_joints <= 0)
    {
        throw std::runtime_error("PID_Kinematic::init: invalid number of joints");
    }

    nq = num_joints;

    target_joint.resize(nq);
    joint_error.resize(nq);
    q_des.resize(nq);
    qdot_des.resize(nq);

    target_joint.setZero();
    joint_error.setZero();
    q_des = target_joint;
    qdot_des.setZero();

    


    // orientation_error and position_error are fixed-size (3)
}


// SET GAINS FUNCTIONS

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

// SET TARGETS FUNCTIONS

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
    if (joint.size() != nq)
    {
        throw std::runtime_error("set_target_joint: size mismatch or init not called");
    }

    this->target_joint = joint;
}

// GET ERRORS FOR FEEDBACK CONTROL

Eigen::Vector3d PID_Kinematic::get_position_error(const Eigen::Vector3d& position_ref, const Eigen::Vector3d& position_cur)
{
    this->position_error = position_ref - position_cur;
    return position_error;
}


Eigen::Vector3d PID_Kinematic::get_orientation_error(const Eigen::Quaterniond& orientation_ref, const Eigen::Quaterniond& orientation_cur)
{
    Eigen::Quaterniond q_error = orientation_ref * orientation_cur.conjugate();
    if (q_error.w() < 0) q_error.coeffs() *= -1.0;
    Eigen::Vector3d v(q_error.x(), q_error.y(), q_error.z());
    double v_norm = v.norm();
    if (v_norm < 1e-8) { orientation_error.setZero(); return orientation_error; }
    double angle = 2.0 * std::acos(q_error.w());
    orientation_error = (v / v_norm) * angle;
    return orientation_error;
}



Eigen::VectorXd PID_Kinematic::get_joint_error(const Eigen::VectorXd& joint_ref, const Eigen::VectorXd& joint_cur)
{
    this->joint_error = joint_ref - joint_cur;
    return joint_error;
}


Eigen::Matrix<double, 6, 1> PID_Kinematic::compute_twist_des(const Eigen::Vector3d& v,
                                 const Eigen::Vector3d& w)
{
    xdot_des.head<3>() = v;
    xdot_des.tail<3>() = w;
    return xdot_des;
}



// compute joint velocity
Eigen::VectorXd PID_Kinematic::compute_joint_velocity_des(
    const Eigen::Matrix<double, 6, 1>& xdot_des,
    const Eigen::MatrixXd& J_pinv)
{
    if (J_pinv.cols() != 6 || J_pinv.rows() != nq)
    {
        throw std::runtime_error("compute_joint_velocity_des: Jacobian size mismatch");
    }

    qdot_des = J_pinv * xdot_des;
    return qdot_des;
}

void PID_Kinematic::integrate_joint_des(double dt)
{
    if (dt <= 0.0)
    {
        throw std::runtime_error("integrate_joint_des: invalid dt");
    }

    if (qdot_des.size() != nq)
    {
        throw std::runtime_error("integrate_joint_des: qdot_des not initialised");
    }

    q_des += qdot_des * dt;
}


Eigen::VectorXd PID_Kinematic::get_q_des() const
{
    return q_des;
}


