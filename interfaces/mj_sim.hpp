#pragma once

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <memory>
#include <vector>
#include <filesystem>



#include "robot_model.hpp"
#include "PID/kinematic_pid.hpp"
#include "PID/PID.hpp"

class MJSim
{
public:
    MJSim(int argc, char** argv);
    ~MJSim();

    void init_control();
    void control_loop_run();

private:
    // lifecycle helpers
    void init_glfw();
    void init_mujoco(int argc, char** argv);
    void init_rendering();

    // camera + rendering
    int get_camera_target_body_id() const;
    void update_camera_lookat(int camera_target_body_id);
    void render_frame();
     

    // GLFW callbacks
    static void mouse_button_cb(GLFWwindow* window, int button, int action, int mods);
    static void cursor_pos_cb(GLFWwindow* window, double xpos, double ypos);
    static void scroll_cb(GLFWwindow* window, double xoffset, double yoffset);

    // MuJoCo state
    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;

    mjvCamera camera_;
    mjvOption options_;
    mjvScene scene_;
    mjrContext context_;

    GLFWwindow* window_ = nullptr;
    bool camera_ready_ = false;

    void load_model_with_search(const std::vector<std::filesystem::path>& search_paths);


    // robot + control
    std::unique_ptr<RobotModel> robot_;

    PID_Kinematic kin_pid_;

    // bookkeeping
    int iter_ = 0;
    double get_dt() const;


    static constexpr double SCROLL_ZOOM_SCALE = 0.05;

    struct MouseState
    {
        bool left = false;
        bool right = false;
        bool middle = false;
        double lastx = 0.0;
        double lasty = 0.0;
    };

    MouseState mouse_;
};
