#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <iostream>
#include <vector>
#include <cmath>

#include "robot_model.hpp"

// MuJoCo model (immutable description of the world)
mjModel* model = nullptr;

// MuJoCo data (mutable simulation state)
mjData* data = nullptr;

// Visualization structures
mjvCamera camera;
mjvOption options;
mjvScene scene;
mjrContext context;

// Define PI explicitly (portable, MSVC-safe)
constexpr double PI = 3.14159265358979323846;

int main()
{
    // 1. Initialise GLFW
    if (!glfwInit())
    {
        std::cerr << "Failed to initialise GLFW" << std::endl;
        return 1;
    }

    // 2. Create window and OpenGL context
    GLFWwindow* window = glfwCreateWindow(
        1200, 900, "MuJoCo C++", nullptr, nullptr
    );
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // 3. Load MuJoCo model
    char error[1000];
    model = mj_loadXML(
        "models/franka/world.xml",
        nullptr,
        error,
        sizeof(error)
    );

    if (!model)
    {
        std::cerr << error << std::endl;
        return 1;
    }

    // 4. Allocate MuJoCo data
    data = mj_makeData(model);

    // 5. Initialise visualisation defaults
    mjv_defaultCamera(&camera);
    mjv_defaultOption(&options);
    mjv_defaultScene(&scene);
    mjr_defaultContext(&context);

    // 6. Allocate scene and context
    mjv_makeScene(model, &scene, 1000);
    mjr_makeContext(model, &context, mjFONTSCALE_150);

    // ------------------------------------------------------------------
    // Construct RobotModel interface
    // ------------------------------------------------------------------
    RobotModel robot(model, data);

    // ------------------------------------------------------------------
    // Set initial joint configuration
    // ------------------------------------------------------------------
    Eigen::VectorXd q0 = Eigen::VectorXd::Zero(robot.get_num_positions());

    // SIMPLE INITIAL CONFIGURATION
    if (q0.size() >= 7)
    {
        q0 << 0.0,
              -PI / 4.0,
               0.0,
              -3.0 * PI / 4.0,
               0.0,
               PI / 2.0,
               PI / 4.0;
    }

    robot.set_joint_positions(q0);
    robot.set_joint_velocities(
        Eigen::VectorXd::Zero(robot.get_num_velocities())
    );

    // Run one forward pass to populate kinematics
    robot.forward();

    // ------------------------------------------------------------------
    // Print bodies and frames (sanity check)
    // ------------------------------------------------------------------
    std::cout << "Bodies:\n";
    robot.print_bodies(std::cout);

    std::cout << "\nFrames (sites):\n";
    robot.print_frames(std::cout);

    // ------------------------------------------------------------------
    // Main simulation loop (no control)
    // ------------------------------------------------------------------
    while (!glfwWindowShouldClose(window))
    {
        // Step physics
        mj_step(model, data);

        // Get framebuffer size
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(
            window, &viewport.width, &viewport.height
        );

        // Update and render scene
        mjv_updateScene(
            model, data, &options, nullptr,
            &camera, mjCAT_ALL, &scene
        );
        mjr_render(viewport, &scene, &context);

        // Swap buffers and poll events
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // ------------------------------------------------------------------
    // Cleanup
    // ------------------------------------------------------------------
    mjr_freeContext(&context);
    mjv_freeScene(&scene);
    mj_deleteData(data);
    mj_deleteModel(model);
    glfwTerminate();

    return 0;
}
