#pragma once
#include <Eigen/Dense>
#include <stdexcept>
#include <string>
#include <vector>
#include <ostream>


class PID_Kinematic
{
    public:
        void set_position_gains(float kp, float ki, float kd);
        void set_orientation_gains(float kp, float ki, float kd);
        void set_joint_gains(float kp, float ki, float kd);


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
        

};