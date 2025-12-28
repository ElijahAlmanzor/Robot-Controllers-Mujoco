#pragma once
#include <Eigen/Dense>
#include <stdexcept>
#include <string>
#include <vector>
#include <ostream>


class PID_Kinematic
{
    public:

        // Set the PID gains - later to do - call the PID objects from custom PID class
        void set_position_gains(float kp, float ki, float kd);
        void set_orientation_gains(float kp, float ki, float kd);
        void set_joint_gains(float kp, float ki, float kd);

        // Set the targets - pose, translation, orientation and desired joint angles
        void set_target_pose(const Eigen::Isometry3d& pose);
        void set_target_position(const Eigen::Vector3d& position);
        void set_target_orientation(const Eigen::Quaterniond& orientation);
        void set_target_joint(const Eigen::VectorXd& joint);



    private:
        // PID gains for the cartesian control for translation
        float hl_pos_Pgain = 0.0f;
        float hl_pos_Igain = 0.0f;
        float hl_pos_Dgain = 0.0f;

        // PID gains for the cartesian control for orientation
        float hl_ori_Pgain = 0.0f;
        float hl_ori_Igain = 0.0f;
        float hl_ori_Dgain = 0.0f;
        
        // PID gains for low-level torque control
        float ll_Pgain = 0.0f;
        float ll_Igain = 0.0f;
        float ll_Dgain = 0.0f;

        // Declare the variables for setting the targets - pose, translation, and orientation and even joint desired angles
        Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
        Eigen::Vector3d target_position = Eigen::Vector3d::Zero();
        Eigen::Quaterniond target_orientation = Eigen::Quaterniond::Identity();
        Eigen::VectorXd target_joint;


        // Internal position, rotation and joint errors!



        

};