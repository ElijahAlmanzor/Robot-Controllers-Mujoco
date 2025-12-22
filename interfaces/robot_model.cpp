// Libraries to load in
#include "robot_model.hpp"

RobotModel::RobotModel(mjModel* model, mjData* data)
    : model_(model), data_(data)
{
}

int RobotModel::get_num_positions() const
{
    // returns number of joint positions from mujoco loaded model
    return model_->nq;
}

int RobotModel::get_num_velocities() const
{
    // returns number of joint velocities from mujoco loaded model
    return model_->nv;
}

Eigen::VectorXd RobotModel::get_joint_positions() const
{
    // Eigen::Map is used to make C array of joint positions into Eigen readable format
    // VectorXd is used for variable sized robot number of configurations
    return Eigen::Map<const Eigen::VectorXd>(data_->qpos, model_->nq);
}

Eigen::VectorXd RobotModel::get_joint_velocities() const
{
    // Eigen::Map is used to make C array of joint positions into Eigen readable format
    // VectorXd is used for variable sized robot number of configurations
    return Eigen::Map<const Eigen::VectorXd>(data_->qvel, model_->nv);
}

void RobotModel::set_joint_positions(const Eigen::VectorXd& q_pos)
{
    // Safety check: correct size
    if (q_pos.size() != model_->nq)
    {
        throw std::runtime_error(
            "set_joint_positions: size mismatch!"
        );
    }

    // Copy joint positions into MuJoCo simulator data states
    for (int i = 0; i < model_->nq; i++)
    {
        data_->qpos[i] = q_pos[i];
    }
}

void RobotModel::set_joint_velocities(const Eigen::VectorXd& q_vel)
{
    // Safety check: correct size
    if (q_vel.size() != model_->nv)
    {
        throw std::runtime_error(
            "set_joint_velocities: size mismatch!"
        );
    }

    // Copy joint velocities into MuJoCo simulator data states
    for (int i = 0; i < model_->nv; i++)
    {
        data_->qvel[i] = q_vel[i];
    }
}

void RobotModel::forward_position()
{
    /*
     * Computes all position-level quantities from the current qpos.
     * Resolves the kinematic tree and updates world-frame positions
     * and orientations of all bodies, joints, sites, and geoms.
     * This is the minimum forward pass required before querying poses.
     */
    mj_fwdPosition(model_, data_);
}

void RobotModel::forward_velocity()
{
    /*
     * Computes all velocity-level quantities from the current qpos and qvel.
     * Propagates spatial velocities through the kinematic tree and updates
     * Jacobians and other velocity-dependent terms.
     * Requires forward_position() to have been called.
     */
    mj_fwdVelocity(model_, data_);
}

void RobotModel::forward_dynamics()
{
    /*
     * Computes actuation and acceleration-level dynamics.
     * Applies actuator forces, computes bias forces, accelerations,
     * and resolves constraint forces.
     * Requires forward_position() and forward_velocity() to have been called.
     */
    mj_fwdActuation(model_, data_);
    mj_fwdAcceleration(model_, data_);
}


void RobotModel::forward()
{
    // Explicit forwarding of everything
    forward_position();
    forward_velocity();
    forward_dynamics();
}

