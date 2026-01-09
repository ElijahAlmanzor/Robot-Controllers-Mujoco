#pragma once

#include <mujoco/mujoco.h>
#include <Eigen/Dense>

#include <stdexcept>
#include <string>
#include <vector>
#include <optional>
#include <ostream>

class RobotModel
{
public:
    RobotModel(mjModel* model, mjData* data);

    // Get values from the robot model
    int get_num_positions() const;   // model_->nq
    int get_num_velocities() const;  // model_->nv
    int get_num_actuators() const; // model->nu

    Eigen::VectorXd get_joint_positions() const;   // data_->qpos
    Eigen::VectorXd get_joint_velocities() const;  // data_->qvel
    

    // Set values for use later by controllers
    void set_joint_positions(const Eigen::VectorXd& q_pos);
    void set_joint_velocities(const Eigen::VectorXd& q_vel);
    void send_torque(const Eigen::VectorXd& tau);


    // Forward computations
    void forward_position();
    void forward_velocity();
    void forward_dynamics();
    void forward();

    // Name extraction of bodies and frames (sites)
    std::vector<std::string> get_body_names() const;
    std::vector<std::string> get_frame_names() const;
    std::optional<int> find_free_joint_id() const;

    // Printing helpers
    void print_bodies(std::ostream& os) const;
    void print_frames(std::ostream& os) const;

    // Kinematics
    Eigen::Isometry3d get_body_pose(const std::string& body_name) const;
    Eigen::Isometry3d get_frame_pose(const std::string& frame_name) const;
    Eigen::MatrixXd get_jacobian(const std::string& name) const;
    Eigen::MatrixXd pinv_jacobian(const Eigen::MatrixXd& jacobian) const;
    
    // Dynamics
    Eigen::VectorXd compute_gravity() const;
    Eigen::VectorXd compute_coriolis() const;
    Eigen::MatrixXd compute_mass_matrix() const;



private:
    mjModel* model_;
    mjData* data_;
};
