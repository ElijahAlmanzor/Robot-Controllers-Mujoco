#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <iostream>

// MuJoCo model (immutable description of the world)
mjModel* model = nullptr;

// MuJoCo data (mutable simulation state)
mjData* data = nullptr;

// Visualization structures (as Global Variables for defining the sim)
mjvCamera camera;
mjvOption options;
mjvScene scene;
mjrContext context;

int main()
{
    // 1. Initialise GLFW (windowing system)
    if (!glfwInit()) {
        std::cerr << "Failed to initialise GLFW" << std::endl;
        return 1;
    }

    // 2. Create a window with an OpenGL context
    GLFWwindow* window = glfwCreateWindow(
        1200, 900, "MuJoCo C++", nullptr, nullptr
    );
    glfwMakeContextCurrent(window); //OpenGL commands apply to the context beloning to this window only
    glfwSwapInterval(1);  // Enable vsync

    // 3. Load MuJoCo model from XML
    char error[1000];
    model = mj_loadXML(
        "models/franka/world.xml",
        nullptr,
        error,
        sizeof(error));
    // bodies, joints, joint types, masses, geometry etc. (immutable)
    
    if (!model) {
        std::cerr << error << std::endl;
        return 1;
    }

    // 4. Allocate simulation state
    data = mj_makeData(model); // robot state, joint poisitons, forces, sensors etc. 

    // 5. Initialise visualization defaults
    mjv_defaultCamera(&camera); // camera position, orientation, tracking mode, zoom etc.
    mjv_defaultOption(&options); // rendering flags
    mjv_defaultScene(&scene); //holds geometry buggers
    mjr_defaultContext(&context); // openGL and GPU stuff

    // 6. Allocate GPU buffers and scene
    mjv_makeScene(model, &scene, 1000);
    mjr_makeContext(model, &context, mjFONTSCALE_150);

    // 7. Main simulation loop
    while (!glfwWindowShouldClose(window)) {

        // Step the physics forward by one timestep
        mj_step(model, data);

        // Get framebuffer size
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(
            window, &viewport.width, &viewport.height
        );

        // Update and render the scene
        mjv_updateScene(
            model, data, &options, nullptr,
            &camera, mjCAT_ALL, &scene
        );
        mjr_render(viewport, &scene, &context);

        // Swap buffers and handle events
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // 8. Cleanup
    mjr_freeContext(&context);
    mjv_freeScene(&scene);
    mj_deleteData(data);
    mj_deleteModel(model);
    glfwTerminate();

    return 0;
}
