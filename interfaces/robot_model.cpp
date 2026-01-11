// Libraries to load in
#include "robot_model.hpp"
#include <iostream>


RobotModel::RobotModel(mjModel* model, mjData* data)
    : model_(model), data_(data)
{
}

RobotModel::RobotModel(
    mjModel* model,
    mjData* data,
    const std::vector<std::string>& arm_joint_names
)
    : RobotModel(model, data)
{
    cache_arm_joint_indices(arm_joint_names);
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

int RobotModel::get_num_actuators() const
{
    // returns number of joint actuators from mujoco loaded model
    return model_->nu;
}

Eigen::VectorXd RobotModel::get_joint_positions() const
{
    return Eigen::Map<const Eigen::VectorXd>(data_->qpos, model_->nq);
}


Eigen::VectorXd RobotModel::get_joint_velocities() const
{
    return Eigen::Map<const Eigen::VectorXd>(data_->qvel, model_->nv);
}


void RobotModel::cache_arm_joint_indices(const std::vector<std::string>& joint_names)
{
    arm_qpos_idx_.clear();
    arm_qvel_idx_.clear();
    arm_qpos_idx_.reserve(joint_names.size());
    arm_qvel_idx_.reserve(joint_names.size());

    for (const auto& jname : joint_names)
    {
        const int jid = mj_name2id(model_, mjOBJ_JOINT, jname.c_str());
        if (jid < 0)
        {
            throw std::runtime_error("Joint not found: " + jname);
        }

        arm_qpos_idx_.push_back(model_->jnt_qposadr[jid]);
        arm_qvel_idx_.push_back(model_->jnt_dofadr[jid]);
    }
}

const std::vector<int>& RobotModel::get_arm_qpos_indices() const
{
    return arm_qpos_idx_;
}

const std::vector<int>& RobotModel::get_arm_qvel_indices() const
{
    return arm_qvel_idx_;
}

Eigen::VectorXd RobotModel::get_arm_joint_positions() const
{
    if (arm_qpos_idx_.empty())
    {
        throw std::runtime_error("Arm joint indices not cached. Call cache_arm_joint_indices() first.");
    }

    Eigen::VectorXd q_arm(static_cast<int>(arm_qpos_idx_.size()));
    for (size_t i = 0; i < arm_qpos_idx_.size(); ++i)
    {
        q_arm[static_cast<int>(i)] = data_->qpos[arm_qpos_idx_[i]];
    }
    return q_arm;
}

Eigen::VectorXd RobotModel::get_arm_joint_velocities() const
{
    if (arm_qvel_idx_.empty())
    {
        throw std::runtime_error("Arm joint indices not cached. Call cache_arm_joint_indices() first.");
    }

    Eigen::VectorXd qdot_arm(static_cast<int>(arm_qvel_idx_.size()));
    for (size_t i = 0; i < arm_qvel_idx_.size(); ++i)
    {
        qdot_arm[static_cast<int>(i)] = data_->qvel[arm_qvel_idx_[i]];
    }
    return qdot_arm;
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

void RobotModel::send_torque(const Eigen::VectorXd& tau)
{
    // Function for sending torque commands to the simulated robot

    int nu = model_->nu;

    // Safety checking
    if (tau.size() > nu)
    {
        throw std::runtime_error(
            "send_torque: torque vector tau is larger than the number of actuators"
        );
    }

    // Zero all actuators first (important!)
    for (int i = 0; i < nu; i++)
    {
        data_->ctrl[i] = 0.0;
    }

    // Write torques for the arm
    for (int i = 0; i < tau.size(); i++)
    {
        data_->ctrl[i] = tau[i];
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
    // Forward simulation of position terms
    mj_fwdPosition(model_, data_);
}

void RobotModel::forward_velocity()
{
    // Forward simulation of joint velocity terms
    mj_fwdVelocity(model_, data_);
}

void RobotModel::forward_dynamics()
{
    // Forward simulation of dynamic terms. 
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

std::vector<std::string> RobotModel::get_body_names() const
{
    std::vector<std::string> names;

    for (int i = 0; i < model_->nbody; ++i)
    {
        const char* name = mj_id2name(model_, mjOBJ_BODY, i);
        if (name && name[0] != '\0')
        {
            names.emplace_back(name);
        }
    }

    return names;
}

std::vector<std::string> RobotModel::get_frame_names() const
{
    std::vector<std::string> names;

    for (int i = 0; i < model_->nsite; ++i)
    {
        const char* name = mj_id2name(model_, mjOBJ_SITE, i);
        if (name && name[0] != '\0')
        {
            names.emplace_back(name);
        }
    }

    return names;
}

std::optional<int> RobotModel::find_free_joint_id() const
{
    for (int j = 0; j < model_->njnt; ++j)
    {
        if (model_->jnt_type[j] == mjJNT_FREE)
        {
            return j;
        }
    }
    return std::nullopt;
}
void RobotModel::print_bodies(std::ostream& os) const
{
    for (const auto& body_name : get_body_names())
    {
        os << body_name << "\n";
    }
}

void RobotModel::print_frames(std::ostream& os) const
{
    for (const auto& frame_name : get_frame_names())
    {
        os << frame_name << "\n";
    }
}


// Functions related to getting poses, jacobians and dynamic terms
Eigen::Isometry3d RobotModel::get_body_pose(const std::string& body_name) const
{
    /* Function for obtaining a particular body's HTM SE(3) */

    // Look up the body id from the body name given by mujoco
    int body_id = mj_name2id(model_, mjOBJ_BODY, body_name.c_str());

    if (body_id < 0)
    {
        throw std::runtime_error("Body name not found: " + body_name);
    }

    // Create the HTM SE(3)
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    // translations and orientations are already in world coordinates
    // Extract the positions (mujoco stores them as 1d flattened vectors)
    Eigen::Vector3d p(
        data_->xpos[3 * body_id + 0],
        data_->xpos[3 * body_id + 1],
        data_->xpos[3 * body_id + 2]
    );

    // Extract orientations as quaternions
    Eigen::Quaterniond q(
        data_->xquat[4 * body_id + 0],  // w
        data_->xquat[4 * body_id + 1],  // x
        data_->xquat[4 * body_id + 2],  // y
        data_->xquat[4 * body_id + 3]   // z
    );

    q.normalize();

    // Fill the HTM
    T.linear()      = q.toRotationMatrix();
    T.translation() = p;

    return T;
}


Eigen::Isometry3d RobotModel::get_frame_pose(const std::string& frame_name) const
{
    /* Function for obtaining a particular frame's pose as SE(3) HTM */

    // Look up the frame (site) id from the frame name given by mujoco
    int frame_id = mj_name2id(model_, mjOBJ_SITE, frame_name.c_str());

    if (frame_id < 0)
    {
        throw std::runtime_error("Frame name not found: " + frame_name);
    }

    // Create the HTM SE(3)
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    // Extract the position (mujoco stores them as 1d flattened vectors)
    Eigen::Vector3d p(
        data_->site_xpos[3 * frame_id + 0],
        data_->site_xpos[3 * frame_id + 1],
        data_->site_xpos[3 * frame_id + 2]
    );

    // Extract the orientation (mujoco stores site rotations as 3x3 matrices)
    Eigen::Matrix3d R;
    R << data_->site_xmat[9 * frame_id + 0], data_->site_xmat[9 * frame_id + 1], data_->site_xmat[9 * frame_id + 2],
         data_->site_xmat[9 * frame_id + 3], data_->site_xmat[9 * frame_id + 4], data_->site_xmat[9 * frame_id + 5],
         data_->site_xmat[9 * frame_id + 6], data_->site_xmat[9 * frame_id + 7], data_->site_xmat[9 * frame_id + 8];

    // Fill the HTM
    T.linear()      = R;
    T.translation() = p;

    return T;
}

Eigen::MatrixXd RobotModel::get_jacobian(const std::string& name) const
{
    /* Function for extracting jacobians out of either bodies or frames! */

    int nv = model_->nv;

    // Allocate raw buffers
    std::vector<mjtNum> Jv(3 * nv);
    std::vector<mjtNum> Jw(3 * nv);

    // Try site (frame) first
    int site_id = mj_name2id(model_, mjOBJ_SITE, name.c_str());

    if (site_id >= 0)
    {
        mj_jacSite(model_, data_, Jv.data(), Jw.data(), site_id);
    }
    else
    {
        // Fallback to body if frame doesn't exist
        int body_id = mj_name2id(model_, mjOBJ_BODY, name.c_str());

        if (body_id >= 0)
        {
            mj_jacBody(model_, data_, Jv.data(), Jw.data(), body_id);
        }
        else
        {
            throw std::runtime_error("No site or body named: " + name);
        }
    }

    // Stack into Eigen matrix
    Eigen::MatrixXd J(6, nv);

    for (int j = 0; j < nv; ++j)
    {
        J(0, j) = Jv[0 * nv + j];
        J(1, j) = Jv[1 * nv + j];
        J(2, j) = Jv[2 * nv + j];

        J(3, j) = Jw[0 * nv + j];
        J(4, j) = Jw[1 * nv + j];
        J(5, j) = Jw[2 * nv + j];
    }

    return J;
}

Eigen::MatrixXd RobotModel::get_arm_jacobian(const std::string& name) const
{
    if (arm_qvel_idx_.empty())
    {
        throw std::runtime_error("Arm joint indices not cached. Call cache_arm_joint_indices() first.");
    }

    const Eigen::MatrixXd J_full = get_jacobian(name);
    Eigen::MatrixXd J_arm(6, static_cast<int>(arm_qvel_idx_.size()));

    for (size_t i = 0; i < arm_qvel_idx_.size(); ++i)
    {
        const int col = arm_qvel_idx_[i];
        if (col < 0 || col >= J_full.cols())
        {
            throw std::runtime_error("Arm velocity index out of range while slicing Jacobian.");
        }
        J_arm.col(static_cast<int>(i)) = J_full.col(col);
    }

    return J_arm;
}

Eigen::MatrixXd RobotModel::pinv_jacobian(
    const Eigen::MatrixXd& jacobian
) const
{
    return jacobian.transpose()
        * (jacobian * jacobian.transpose()
           + 1e-6 * Eigen::MatrixXd::Identity(6, 6)).inverse();
}


            

// Functions for extracting dynamic terms

Eigen::VectorXd RobotModel::compute_gravity() const
{
    int nv = model_->nv;

    // Save current velocities and accelerations
    Eigen::VectorXd qvel_saved =
        Eigen::Map<Eigen::VectorXd>(data_->qvel, nv);
    Eigen::VectorXd qacc_saved =
        Eigen::Map<Eigen::VectorXd>(data_->qacc, nv);

    // Zero velocity and acceleration
    for (int i = 0; i < nv; ++i)
    {
        data_->qvel[i] = 0.0;
        data_->qacc[i] = 0.0;
    }

    // Inverse dynamics computes g(q)
    mj_inverse(model_, data_);

    Eigen::VectorXd g(nv);
    for (int i = 0; i < nv; ++i)
    {
        g[i] = data_->qfrc_inverse[i];
    }

    // Restore state
    for (int i = 0; i < nv; ++i)
    {
        data_->qvel[i] = qvel_saved[i];
        data_->qacc[i] = qacc_saved[i];
    }

    return g;
}

Eigen::VectorXd RobotModel::compute_arm_gravity() const
{
    if (arm_qvel_idx_.empty())
    {
        throw std::runtime_error("Arm joint indices not cached. Call cache_arm_joint_indices() first.");
    }

    const Eigen::VectorXd g_full = compute_gravity();
    Eigen::VectorXd g_arm(static_cast<int>(arm_qvel_idx_.size()));

    for (size_t i = 0; i < arm_qvel_idx_.size(); ++i)
    {
        const int idx = arm_qvel_idx_[i];
        if (idx < 0 || idx >= g_full.size())
        {
            throw std::runtime_error("Arm velocity index out of range while slicing gravity.");
        }
        g_arm[static_cast<int>(i)] = g_full[idx];
    }

    return g_arm;
}

Eigen::VectorXd RobotModel::initialise_joint_configuration(const Eigen::VectorXd& q_arm_init) const
{
    /*
        Why this exists:
        Controllers operate on arm-only joint vectors, but MuJoCo expects a full qpos (size nq).
        If the model has a free joint, qpos contains a base quaternion; leaving it all-zeros is
        an invalid orientation, so we set w=1.0 (identity quaternion) before inserting the arm
        joint values into the correct qpos slots.
    */
    if (arm_qpos_idx_.empty())
    {
        throw std::runtime_error("Arm joint indices not cached. Call cache_arm_joint_indices() first.");
    }
    if (q_arm_init.size() != static_cast<int>(arm_qpos_idx_.size()))
    {
        throw std::runtime_error("initialise_joint_configuration: arm init size mismatch.");
    }

    const int nq = model_->nq;
    Eigen::VectorXd q0 = Eigen::VectorXd::Zero(nq);

    if (std::optional<int> free_joint_id = find_free_joint_id();
        free_joint_id.has_value())
    {
        const int adr = model_->jnt_qposadr[free_joint_id.value()];
        if (adr + 3 < nq)
        {
            q0[adr + 3] = 1.0;  // quaternion w = 1
        }
    }

    for (size_t i = 0; i < arm_qpos_idx_.size(); ++i)
    {
        const int idx = arm_qpos_idx_[i];
        if (idx < 0 || idx >= nq)
        {
            throw std::runtime_error("initialise_joint_configuration: arm position index out of range.");
        }
        q0[idx] = q_arm_init[static_cast<int>(i)];
    }

    return q0;
}



Eigen::VectorXd RobotModel::compute_coriolis() const
{
    Eigen::VectorXd g = compute_gravity();

    int nv = model_->nv;
    Eigen::VectorXd c(nv);

    for (int i = 0; i < nv; ++i)
    {
        c[i] = data_->qfrc_bias[i] - g[i];
    }

    return c;
}


Eigen::MatrixXd RobotModel::compute_mass_matrix() const
{
    // Joint space mass matrix M(q)
    // Size: nv x nv
    // MuJoCo stores this in compressed form and provides mj_fullM to expand it

    int nv = model_->nv;

    // Temporary buffer for full mass matrix (row-major, flat)
    std::vector<mjtNum> M_full(nv * nv);

    // Expand compressed mass matrix data_->qM into full form
    mj_fullM(model_, M_full.data(), data_->qM);

    // Copy into Eigen matrix
    Eigen::MatrixXd M(nv, nv);
    for (int i = 0; i < nv; ++i)
    {
        for (int j = 0; j < nv; ++j)
        {
            M(i, j) = M_full[i * nv + j];
        }
    }

    return M;
}
