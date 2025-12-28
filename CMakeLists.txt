cmake_minimum_required(VERSION 4.2.1)
project(control_demos)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(MUJOCO_DIR "C:/mujoco")

find_package(glfw3 CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

# ------------------------------------------------------------------
# Interfaces library
# ------------------------------------------------------------------
add_library(interfaces
    interfaces/robot_model.cpp
)

target_include_directories(interfaces PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/interfaces
    ${MUJOCO_DIR}/include
)

target_link_libraries(interfaces PUBLIC
    ${MUJOCO_DIR}/lib/mujoco.lib
    Eigen3::Eigen
)

# ------------------------------------------------------------------
# Controllers library (PID)
# ------------------------------------------------------------------
add_library(controllers
    controllers/PID/kinematic_pid.cpp
    controllers/PID/PID.cpp
)

target_include_directories(controllers PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/controllers
    ${MUJOCO_DIR}/include
)

target_link_libraries(controllers PUBLIC
    interfaces          # controllers depend on robot_model
    Eigen3::Eigen
)

# ------------------------------------------------------------------
# Executable
# ------------------------------------------------------------------
add_executable(control_demos_app
    src/main.cpp
)

target_link_libraries(control_demos_app PRIVATE
    interfaces
    controllers
    glfw
)

# ------------------------------------------------------------------
# Runtime DLL copy (Windows)
# ------------------------------------------------------------------
add_custom_command(TARGET control_demos_app POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "C:/vcpkg/installed/x64-windows/bin/glfw3.dll"
        "C:/mujoco/bin/mujoco.dll"
        $<TARGET_FILE_DIR:control_demos_app>
)

# ------------------------------------------------------------------
# Install rules
# ------------------------------------------------------------------
install(TARGETS control_demos_app
    RUNTIME DESTINATION bin
)

install(FILES
    "C:/vcpkg/installed/x64-windows/bin/glfw3.dll"
    "C:/mujoco/bin/mujoco.dll"
    DESTINATION bin
)

install(DIRECTORY models/
    DESTINATION share/control_demos/models
)
