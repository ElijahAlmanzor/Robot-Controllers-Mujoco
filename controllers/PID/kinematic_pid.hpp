#pragma once

#include <Eigen/Dense>
#include <stdexcept>
#include <cmath>
#include <algorithm>



class PID_Kinematic
{
    public:

        // Initialise controller dimensions (must be called once)
        // Typically called after robot_model is created
        void init(int num_joints);

        // Set PID gains for high-level Cartesian position control
        void set_position_gains(float kp, float ki, float kd);

        // Set PID gains for high-level Cartesian orientation control
        void set_orientation_gains(float kp, float ki, float kd);

        // Set PID gains for low-level joint space control
        void set_joint_gains(float kp, float ki, float kd);

        // Set target pose in Cartesian space (HTM)
        void set_target_pose(const Eigen::Isometry3d& pose);

        // Set target position in Cartesian space
        void set_target_position(const Eigen::Vector3d& position);

        // Set target orientation in Cartesian space
        void set_target_orientation(const Eigen::Quaterniond& orientation);

        // Set target joint configuration for low-level controller
        void set_target_joint(const Eigen::VectorXd& joint);

        // Compute Cartesian position error (ref - current)
        Eigen::Vector3d get_position_error(const Eigen::Vector3d& position_ref, const Eigen::Vector3d& position_cur);

        // Compute Cartesian orientation error (stored as so(3) vector)
        Eigen::Vector3d get_orientation_error(const Eigen::Quaterniond& orientation_ref, const Eigen::Quaterniond& orientation_cur);

        // Compute joint space error (ref - current)
        Eigen::VectorXd get_joint_error(const Eigen::VectorXd& joint_ref, const Eigen::VectorXd& joint_cur);

        // stack the control outputs
        Eigen::Matrix<double, 6, 1> compute_twist_des(const Eigen::Vector3d& v, const Eigen::Vector3d& w);

        // compute the desired joint velocity
        // Map desired Cartesian twist to desired joint velocity
        Eigen::VectorXd compute_joint_velocity_des(const Eigen::Matrix<double, 6, 1>& xdot_des, const Eigen::MatrixXd& J_pinv);


        // Integrate desired joint velocity to update desired joint position
        void integrate_joint_des(double dt);

        // get q_des
        Eigen::VectorXd get_q_des() const;

        
    private:

        // Number of joints (set during init)
        int nq = 0;
        Eigen::VectorXd q_des;       // desired joint position (state)
        Eigen::VectorXd qdot_des;    // desired joint velocity (optional but useful)

        // For combining the different position and orienation PID terms
        Eigen::Matrix<double, 6, 1> xdot_des;


        // PID gains for high-level Cartesian position control
        float hl_pos_Pgain = 0.0f;
        float hl_pos_Igain = 0.0f;
        float hl_pos_Dgain = 0.0f;

        // PID gains for high-level Cartesian orientation control
        float hl_ori_Pgain = 0.0f;
        float hl_ori_Igain = 0.0f;
        float hl_ori_Dgain = 0.0f;

        // PID gains for low-level joint space torque control
        float ll_Pgain = 0.0f;
        float ll_Igain = 0.0f;
        float ll_Dgain = 0.0f;

        // Target representations
        Eigen::Isometry3d  target_pose        = Eigen::Isometry3d::Identity();
        Eigen::Vector3d    target_position    = Eigen::Vector3d::Zero();
        Eigen::Quaterniond target_orientation = Eigen::Quaterniond::Identity();
        Eigen::VectorXd    target_joint;

        // Internal error states
        Eigen::Vector3d position_error    = Eigen::Vector3d::Zero();
        Eigen::Vector3d orientation_error = Eigen::Vector3d::Zero();
        Eigen::VectorXd joint_error;
        
};
