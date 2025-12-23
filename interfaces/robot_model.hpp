#pragma once

#include <mujoco/mujoco.h>
#include <Eigen/Dense>
#include <stdexcept>
#include <string>
#include <vector>
#include <ostream>

class RobotModel
{
public:
    RobotModel(mjModel* model, mjData* data);

    // Get values from the robot model
    int get_num_positions() const;   // model->nq, number of joint positions
    int get_num_velocities() const;  // model->nv, number of joint velocities

    Eigen::VectorXd get_joint_positions() const;   // data->qpos
    Eigen::VectorXd get_joint_velocities() const;  // data->qvel
    
    // name extraction of bodies and sites with printing
    std::vector<std::string> get_body_names() const;
    std::vector<std::string> get_frame_names() const;
    void print_bodies() const; // ostream as only outputting
    void print_frames() const;

    // Set values for use later by controllers
    void set_joint_positions(const Eigen::VectorXd& q_pos);
    void set_joint_velocities(const Eigen::VectorXd& q_vel);

    // Forward computations
    void forward_position();
    void forward_velocity();
    void forward_dynamics();
    void forward(); // complete forwards for kinematics and dynamics

private:
    // Core pointers to important mujoco variables
    mjModel* model_;
    mjData* data_;

    // name extraction of bodies and sites with printing
    std::vector<std::string> get_body_names() const;
    std::vector<std::string> get_frame_names() const;
};
