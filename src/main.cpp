#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <filesystem>
#include <optional>
#include <sstream>

#include "robot_model.hpp"
#include "PID/kinematic_pid.hpp"
#include "PID/PID.hpp"


// MuJoCo globals
mjModel* model = nullptr;
mjData* data = nullptr;

mjvCamera camera;
mjvOption options;
mjvScene scene;
mjrContext context;

constexpr double PI = 3.14159265358979323846;
constexpr int ARM_DOF = 7;

mjModel* load_model_with_search(
    const std::vector<std::filesystem::path>& search_paths,
    char* error_buf,
    int error_buf_size)
{
    for (const auto& path : search_paths)
    {
        error_buf[0] = '\0';

        mjModel* m = mj_loadXML(
            path.string().c_str(),
            nullptr, error_buf, error_buf_size
        );

        if (m)
        {
            std::cout << "Loaded model: " << path.string() << "\n";
            return m;
        }

        std::cerr << "Failed to load model at "
                  << path.string() << " : "
                  << error_buf << "\n";
    }

    return nullptr;
}

std::optional<int> find_free_joint_id(const mjModel* m)
{
    for (int j = 0; j < m->njnt; ++j)
    {
        if (m->jnt_type[j] == mjJNT_FREE)
        {
            return j;
        }
    }
    return std::nullopt;
}

int main(int argc, char** argv)
{
    // GLFW init
    if (!glfwInit())
    {
        std::cerr << "Failed to initialise GLFW\n";
        return 1;
    }
    std::cout << "GLFW initialised\n";

    GLFWwindow* window = glfwCreateWindow(
        1200, 900, "MuJoCo C++", nullptr, nullptr
    );
    if (!window)
    {
        std::cerr << "Failed to create GLFW window\n";
        return 1;
    }
    std::cout << "GLFW window created\n";

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // Build model search paths (works for run-from-source and installed tree)
    std::filesystem::path exe_dir = (argc > 0)
        ? std::filesystem::absolute(argv[0]).parent_path()
        : std::filesystem::current_path();

    std::vector<std::filesystem::path> model_search_paths = {
        std::filesystem::path("models/franka/world.xml"),                        // run from repo root
        exe_dir / "models/franka/world.xml",                                     // run from build dir
        exe_dir / ".." / "share" / "control_demos" / "models" / "franka" / "world.xml" // installed layout
    };

    // Load MuJoCo model
    char error[1000];
    model = load_model_with_search(model_search_paths, error, sizeof(error));

    if (!model)
    {
        std::cerr << error << "\n";
        return 1;
    }
    std::cout << "mj_loadXML succeeded\n";

    data = mj_makeData(model);
    if (!data)
    {
        std::cerr << "mj_makeData failed\n";
        return 1;
    }
    std::cout << "mj_makeData succeeded\n";

    // Identify arm joint indices (qpos and qvel)
    const std::vector<std::string> arm_joint_names = {
        "panda_joint1", "panda_joint2", "panda_joint3",
        "panda_joint4", "panda_joint5", "panda_joint6",
        "panda_joint7"
    };

    std::vector<int> arm_qpos_idx;
    std::vector<int> arm_qvel_idx;
    arm_qpos_idx.reserve(ARM_DOF);
    arm_qvel_idx.reserve(ARM_DOF);

    for (const auto& jname : arm_joint_names)
    {
        int jid = mj_name2id(model, mjOBJ_JOINT, jname.c_str());
        if (jid < 0)
        {
            std::cerr << "Joint not found: " << jname << "\n";
            return 1;
        }
        arm_qpos_idx.push_back(model->jnt_qposadr[jid]);
        arm_qvel_idx.push_back(model->jnt_dofadr[jid]);
    }
    std::cout << "Arm joint indices resolved\n";

    // Free joint (base) quaternion must be valid
    std::optional<int> free_joint_id = find_free_joint_id(model);

    mjv_defaultCamera(&camera);
    mjv_defaultOption(&options);
    mjv_defaultScene(&scene);
    mjr_defaultContext(&context);

    mjv_makeScene(model, &scene, 1000);
    mjr_makeContext(model, &context, mjFONTSCALE_150);

    // Robot interface
    RobotModel robot(model, data);

    const int nq = robot.get_num_positions();
    const int nv = robot.get_num_velocities();
    const int nu = model->nu;

    const std::string ee_name = "panda_hand";  // end-effector body from XML

    // Initial joint configuration (arm only)
    Eigen::VectorXd q0 = Eigen::VectorXd::Zero(nq);

    // Ensure a valid base quaternion if the model has a free joint
    if (free_joint_id.has_value())
    {
        int adr = model->jnt_qposadr[free_joint_id.value()];
        if (adr + 3 < nq)
        {
            q0[adr + 3] = 1.0;  // quaternion w = 1
        }
    }

    const Eigen::Matrix<double, ARM_DOF, 1> q_arm_init = (Eigen::Matrix<double, ARM_DOF, 1>() <<
        0.0,
        -PI / 4.0,
        0.0,
        -3.0 * PI / 4.0,
        0.0,
        PI / 2.0,
        PI / 4.0).finished();

    for (int i = 0; i < ARM_DOF; ++i)
    {
        q0[arm_qpos_idx[i]] = q_arm_init[i];
    }
    std::cout << "Initial joint positions set\n";

    robot.set_joint_positions(q0);
    robot.set_joint_velocities(Eigen::VectorXd::Zero(nv));
    robot.forward();
    std::cout << "Initial robot forward pass complete\n";

    std::cout << "Model loaded. nq=" << nq
              << " nv=" << nv
              << " nu=" << nu << "\n";
    std::cout << "Arm qpos idx: ";
    for (int idx : arm_qpos_idx) std::cout << idx << " ";
    std::cout << "\nArm qvel idx: ";
    for (int idx : arm_qvel_idx) std::cout << idx << " ";
    std::cout << "\nEnd-effector body: " << ee_name << "\n";

    // Kinematic helper
    PID_Kinematic kin_pid;
    kin_pid.init(ARM_DOF);

    // Task-space PIDs
    PID pid_pos;
    pid_pos.init(3);
    pid_pos.set_gains(10.5, 0.0, 0.0);

    PID pid_ori;
    pid_ori.init(3);
    pid_ori.set_gains(10.0, 0.0, 0.0);

    // Joint-space PID
    PID pid_joint;
    pid_joint.init(nq);
    pid_joint.set_gains(80.0, 5.0, 5.0);
    pid_joint.set_integral_limit(10.0);

    // Target pose
    Eigen::Isometry3d T_target = Eigen::Isometry3d::Identity();
    T_target.translation() << 0.5, 0.3, 0.5;
    T_target.linear() =
        Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitX()).toRotationMatrix();

    kin_pid.set_target_pose(T_target);
    kin_pid.set_target_joint(q_arm_init);

    int iter = 0;

    auto print_state = [&](const Eigen::VectorXd& q_arm,
                           const Eigen::Vector3d& pos_err,
                           const Eigen::Vector3d& ori_err,
                           const Eigen::VectorXd& tau)
    {
        std::ostringstream oss;
        oss << "[iter " << iter << "] "
            << "q_arm: " << q_arm.transpose()
            << " | pos_err: " << pos_err.transpose()
            << " | ori_err: " << ori_err.transpose()
            << " | tau: " << tau.transpose();
        std::cout << oss.str() << "\n";
    };

    try
    {
        // Main loop
        while (!glfwWindowShouldClose(window))
        {
            if (iter == 0)
            {
                std::cout << "Entering main loop\n";
            }
            mj_step(model, data);
            robot.forward();

            const double dt = model->opt.timestep;

            // Read robot state
            Eigen::VectorXd q = robot.get_joint_positions();
            Eigen::VectorXd qdot = robot.get_joint_velocities();

            // Extract arm state (ignore free joint + fingers)
            Eigen::VectorXd q_arm(ARM_DOF);
            Eigen::VectorXd qdot_arm(ARM_DOF);
            for (int i = 0; i < ARM_DOF; ++i)
            {
                q_arm[i] = q[arm_qpos_idx[i]];
                qdot_arm[i] = qdot[arm_qvel_idx[i]];
            }

            Eigen::Isometry3d T_W_ee = robot.get_body_pose(ee_name);
            Eigen::MatrixXd J = robot.get_jacobian(ee_name);

            // Slice Jacobian to arm DOFs
            Eigen::MatrixXd J_arm(6, ARM_DOF);
            for (int i = 0; i < ARM_DOF; ++i)
            {
                J_arm.col(i) = J.col(arm_qvel_idx[i]);
            }

            // Compute task-space errors
            Eigen::Vector3d pos_err =
                kin_pid.get_position_error(
                    T_target.translation(),
                    T_W_ee.translation()
                );

            Eigen::Vector3d ori_err =
                kin_pid.get_orientation_error(
                    Eigen::Quaterniond(T_target.linear()),
                    Eigen::Quaterniond(T_W_ee.linear())
                );

            // Task-space PID -> twist
            Eigen::Vector3d v_cmd = pid_pos.all_terms(pos_err, dt);
            Eigen::Vector3d w_cmd = pid_ori.all_terms(ori_err, dt);

            Eigen::Matrix<double, 6, 1> xdot_des =
                kin_pid.compute_twist_des(v_cmd, w_cmd);

            // Jacobian pseudoinverse
            Eigen::MatrixXd J_pinv =
                J_arm.transpose()
                * (J_arm * J_arm.transpose()
                   + 1e-6 * Eigen::MatrixXd::Identity(6, 6)).inverse();

            // Map to joint space
            kin_pid.compute_joint_velocity_des(xdot_des, J_pinv);

            // Integrate desired joint state
            kin_pid.integrate_joint_des(dt);

            // Joint-space PID -> torque
            Eigen::VectorXd e_q =
                kin_pid.get_joint_error(kin_pid.get_q_des(), q_arm);

            // Slice gravity to arm DOFs
            Eigen::VectorXd g = robot.compute_gravity();
            Eigen::VectorXd g_arm(ARM_DOF);
            for (int i = 0; i < ARM_DOF; ++i)
            {
                g_arm[i] = g[arm_qvel_idx[i]];
            }

            Eigen::VectorXd tau =
                pid_joint.all_terms(e_q, dt)
            + g_arm;



            robot.send_torque(tau);

            if (iter < 10)
            {
                print_state(q_arm, pos_err, ori_err, tau);
            }

            ++iter;

            // Render
            mjrRect viewport = {0, 0, 0, 0};
            glfwGetFramebufferSize(
                window, &viewport.width, &viewport.height
            );

            mjv_updateScene(
                model, data, &options, nullptr,
                &camera, mjCAT_ALL, &scene
            );
            mjr_render(viewport, &scene, &context);

            glfwSwapBuffers(window);
            glfwPollEvents();
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Fatal error: " << e.what() << "\n";
    }


    // Cleanup
    mjr_freeContext(&context);
    mjv_freeScene(&scene);
    mj_deleteData(data);
    mj_deleteModel(model);
    glfwTerminate();

    return 0;
}
