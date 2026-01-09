#include "mj_sim.hpp"
#include <iostream>
#include <optional>

#include "franka_sim_setup.hpp"


MJSim::MJSim(int argc, char** argv)
{
    init_glfw();
    init_mujoco(argc, argv);
    init_rendering();
    init_control();
}


void MJSim::init_glfw()
{
    if (!glfwInit())
    {
        std::cerr << "Failed to initialise GLFW\n";
    }

    // Else say, it's been initalised
    std::cout << "GLFW initialised\n";

    // Create the window for disaply
    window_ = glfwCreateWindow(
    1200, 900, "MuJoCo C++", nullptr, nullptr
    );
    
    if (!window_)
    {
        std::cerr << "Failed to create GLFW window\n";
    }
    std::cout << "GLFW window created\n"; 

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    glfwSetWindowUserPointer(window_, this);
    glfwSetMouseButtonCallback(window_, mouse_button_cb);
    glfwSetCursorPosCallback(window_, cursor_pos_cb);
    glfwSetScrollCallback(window_, scroll_cb);

}


void MJSim::mouse_button_cb(GLFWwindow* window, int button, int act, int /*mods*/)
{
    auto* sim = static_cast<MJSim*>(glfwGetWindowUserPointer(window));
    if (!sim || !sim->camera_ready_)
    {
        return;
    }

    auto& ms = sim->mouse_;

    if (button == GLFW_MOUSE_BUTTON_LEFT)
        ms.left = (act == GLFW_PRESS);
    else if (button == GLFW_MOUSE_BUTTON_RIGHT)
        ms.right = (act == GLFW_PRESS);
    else if (button == GLFW_MOUSE_BUTTON_MIDDLE)
        ms.middle = (act == GLFW_PRESS);

    glfwGetCursorPos(window, &ms.lastx, &ms.lasty);
}

void MJSim::cursor_pos_cb(GLFWwindow* window, double xpos, double ypos)
{
    auto* sim = static_cast<MJSim*>(glfwGetWindowUserPointer(window));
    if (!sim)
        return;

    auto& ms = sim->mouse_;

    int width = 0, height = 0;
    glfwGetWindowSize(window, &width, &height);
    if (height <= 0)
        return;

    const double dx = (xpos - ms.lastx) / static_cast<double>(height);
    const double dy = (ypos - ms.lasty) / static_cast<double>(height);
    ms.lastx = xpos;
    ms.lasty = ypos;

    if (!(ms.left || ms.right || ms.middle))
        return;

    mjv_moveCamera(sim->model_, mjMOUSE_ROTATE_H, dx, dy, &sim->scene_, &sim->camera_);
}


void MJSim::scroll_cb(GLFWwindow* window, double /*xoffset*/, double yoffset)
{
    auto* sim = static_cast<MJSim*>(glfwGetWindowUserPointer(window));
    if (!sim || !sim->camera_ready_)
        return;

    mjv_moveCamera(
        sim->model_,
        mjMOUSE_ZOOM,
        0.0,
        SCROLL_ZOOM_SCALE * yoffset,
        &sim->scene_,
        &sim->camera_
    );
}


void MJSim::init_mujoco(int argc, char** argv)
{
    std::filesystem::path exe_dir = (argc > 0)
        ? std::filesystem::absolute(argv[0]).parent_path()
        : std::filesystem::current_path();

    std::vector<std::filesystem::path> model_search_paths = {
        std::filesystem::path("models/franka/world.xml"),
        exe_dir / "models/franka/world.xml",
        exe_dir / ".." / "share" / "control_demos" / "models" / "franka" / "world.xml"
    };

    load_model_with_search(model_search_paths);

    data_ = mj_makeData(model_);
    if (!data_)
    {
        throw std::runtime_error("mj_makeData failed");
    }
}


void MJSim::load_model_with_search(
    const std::vector<std::filesystem::path>& search_paths)
{
    char error[1000];

    for (const auto& path : search_paths)
    {
        error[0] = '\0';

        mjModel* m = mj_loadXML(
            path.string().c_str(),
            nullptr,
            error,
            sizeof(error)
        );

        if (m)
        {
            std::cout << "Loaded model: " << path.string() << "\n";
            model_ = m;
            return;
        }

        std::cerr << "Failed to load model at "
                  << path.string() << " : "
                  << error << "\n";
    }

    throw std::runtime_error("Failed to load MuJoCo model");
}


void MJSim::init_rendering()
{
    // initialise MuJoCo visual state
    mjv_defaultCamera(&camera_);
    mjv_defaultOption(&options_);
    mjv_defaultScene(&scene_);
    mjr_defaultContext(&context_);

    // create scene and rendering context
    mjv_makeScene(model_, &scene_, 1000);
    mjr_makeContext(model_, &context_, mjFONTSCALE_150);

    // camera and scene are now safe to use
    camera_ready_ = true;
}



MJSim::~MJSim()
{
    //Program exterminator
    // If RobotModel stores pointers into MuJoCo, drop it first
    robot_.reset();

    // Free MuJoCo rendering resources
    mjr_freeContext(&context_);
    mjv_freeScene(&scene_);

    // Free MuJoCo simulation data
    if (data_)
    {
        mj_deleteData(data_);
        data_ = nullptr;
    }

    if (model_)
    {
        mj_deleteModel(model_);
        model_ = nullptr;
    }

    // Destroy GLFW window if created
    if (window_)
    {
        glfwDestroyWindow(window_);
        window_ = nullptr;
    }

    // Terminate GLFW if it was initialised
    // This is safe to call even if glfwInit failed, but guarding is fine too.
    glfwTerminate();
}


void MJSim::init_control()
{
    //values taken from franka_sim_setup.hpp
    kin_pid_.init(ARM_DOF);
    
    kin_pid_.set_position_gains(PID_POS_KP, PID_POS_KI, PID_POS_KD);
    kin_pid_.set_orientation_gains(PID_ORI_KP, PID_ORI_KI, PID_ORI_KD);
    kin_pid_.set_joint_gains(PID_JOINT_KP, PID_JOINT_KI, PID_JOINT_KD);
    kin_pid_.set_joint_integral_limit(PID_JOINT_I_LIMIT);

    kin_pid_.set_target_pose(T_TARGET);
    kin_pid_.set_target_joint(Q_ARM_INIT);
}

/* LAST DO STEPS BEFORE I BUILD OTHGER CONTROLLERS

1. Clean up control_loop_run with more abstractions
2. Add a simple switch case to choose controllers for scalability! 

*/
void MJSim::control_loop_run()
{
    if (!robot_)
    {
        robot_ = std::make_unique<RobotModel>(model_, data_);
    }

    // Precompute arm indexing (MuJoCo model-dependent; controller-independent)
    std::vector<int> arm_qpos_idx;
    std::vector<int> arm_qvel_idx;
    arm_qpos_idx.reserve(ARM_DOF);
    arm_qvel_idx.reserve(ARM_DOF);

    for (const auto& jname : FRANKA_ARM_JOINT_NAMES)
    {
        int jid = mj_name2id(model_, mjOBJ_JOINT, jname.c_str());
        if (jid < 0)
        {
            throw std::runtime_error("Joint not found: " + jname);
        }

        arm_qpos_idx.push_back(model_->jnt_qposadr[jid]);
        arm_qvel_idx.push_back(model_->jnt_dofadr[jid]);
    }

    const int camera_target_body_id =
        mj_name2id(model_, mjOBJ_BODY, "panda_link0");

    // Initial robot state (simulation setup; not controller setup)
    const int nq = robot_->get_num_positions();
    const int nv = robot_->get_num_velocities();

    Eigen::VectorXd q0 = Eigen::VectorXd::Zero(nq);

    if (std::optional<int> free_joint_id = robot_->find_free_joint_id();
        free_joint_id.has_value())
    {
        int adr = model_->jnt_qposadr[free_joint_id.value()];
        if (adr + 3 < nq)
        {
            q0[adr + 3] = 1.0;  // quaternion w = 1
        }
    }

    for (int i = 0; i < ARM_DOF; ++i)
    {
        q0[arm_qpos_idx[i]] = Q_ARM_INIT[i];
    }

    robot_->set_joint_positions(q0);
    robot_->set_joint_velocities(Eigen::VectorXd::Zero(nv));
    robot_->forward();

    while (!glfwWindowShouldClose(window_))
    {
        mj_step(model_, data_);
        robot_->forward();

        const double dt = model_->opt.timestep;

        // Full robot state
        const Eigen::VectorXd q_full = robot_->get_joint_positions();
        const Eigen::VectorXd qdot_full = robot_->get_joint_velocities();

        // Slice arm state (arm-only, controller input)
        Eigen::VectorXd q_arm(ARM_DOF);
        Eigen::VectorXd qdot_arm(ARM_DOF);
        for (int i = 0; i < ARM_DOF; ++i)
        {
            q_arm[i] = q_full[arm_qpos_idx[i]];
            qdot_arm[i] = qdot_full[arm_qvel_idx[i]];
        }

        const Eigen::Isometry3d T_W_ee = robot_->get_body_pose(EE_NAME);

        // Slice Jacobian columns to arm DOFs
        const Eigen::MatrixXd J_full = robot_->get_jacobian(EE_NAME);
        Eigen::MatrixXd J_arm(6, ARM_DOF);
        for (int i = 0; i < ARM_DOF; ++i)
        {
            J_arm.col(i) = J_full.col(arm_qvel_idx[i]);
        }

        // Slice gravity to arm DOFs
        const Eigen::VectorXd g_full = robot_->compute_gravity();
        Eigen::VectorXd g_arm(ARM_DOF);
        for (int i = 0; i < ARM_DOF; ++i)
        {
            g_arm[i] = g_full[arm_qvel_idx[i]];
        }

        // One controller call per step (arm-only in/out)
        const Eigen::VectorXd tau_arm =
            kin_pid_.compute(dt, q_arm, qdot_arm, T_W_ee, J_arm, g_arm);

        robot_->send_torque(tau_arm);

        if (camera_target_body_id >= 0)
        {
            const mjtNum* base_pos = data_->xpos + 3 * camera_target_body_id;
            camera_.lookat[0] = base_pos[0];
            camera_.lookat[1] = base_pos[1];
            camera_.lookat[2] = base_pos[2];
        }

        // Render
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

        mjv_updateScene(
            model_, data_, &options_, nullptr,
            &camera_, mjCAT_ALL, &scene_
        );
        mjr_render(viewport, &scene_, &context_);

        glfwSwapBuffers(window_);
        glfwPollEvents();

        ++iter_;
    }
}
