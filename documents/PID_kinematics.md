# PID Kinematics with Gravity Compensation

## Overview
This note documents the math used by the kinematic PID loop in this repo. The controller is a two-layer scheme:

1) Task-space PID on end-effector position and orientation.
2) Map the desired twist to joint velocities with a Jacobian pseudoinverse.
3) Integrate to a desired joint position.
4) Joint-space PID torque plus gravity compensation.

The logic mirrors the flow in `src/main.cpp` and the helpers in `controllers/PID/kinematic_pid.*` and `controllers/PID/PID.*`.

## Notation (matches variable names in code)

| Symbol | Meaning |
| --- | --- |
| $q, \dot{q}$ | Current joint position and velocity (full model or arm slice). |
| $q_{des}, \dot{q}_{des}$ | Desired joint position and velocity (integrated from task space). |
| $p, R$ | End-effector position and rotation in world frame. |
| $p_{ref}, R_{ref}$ | Target end-effector position and rotation. |
| $e_p$ | Position error. |
| $e_o$ | Orientation error (axis-angle vector in $\mathfrak{so}(3)$, radians). |
| $v_{cmd}, w_{cmd}$ | Commanded linear and angular velocity from task-space PID. |
| $\dot{x}_{des}$ | Desired twist $\begin{bmatrix} v_{cmd} \\ w_{cmd} \end{bmatrix}$. |
| $J$ | Geometric Jacobian of end-effector (6 x $n_v$). |
| $J^{\dagger}$ | Damped pseudoinverse of $J$. |
| $e_q$ | Joint-space error $q_{des} - q_{arm}$. |
| $K_p, K_i, K_d$ | PID gains for position/orientation/joint loops (with subscripts). |
| $g(q)$ | Gravity torque vector from inverse dynamics. |
| $\Delta t$ | Control timestep (MuJoCo model->opt.timestep). |

## Step 1: Task-space error

Position error is straight subtraction:

$$
e_p = p_{ref} - p
$$

Orientation error uses the shortest quaternion error and converts to axis-angle:

$$
q_{err} = q_{ref} \otimes q_{cur}^*
$$

$$
\text{if } q_{err,w} < 0,\; q_{err} \leftarrow -q_{err}
$$

$$
v = \begin{bmatrix} q_{err,x} \\ q_{err,y} \\ q_{err,z} \end{bmatrix},\;\;
\theta = 2 \arccos(q_{err,w})
$$

$$
e_o =
\begin{cases}
0, & \|v\| < \epsilon \\
\dfrac{v}{\|v\|}\,\theta, & \text{otherwise}
\end{cases}
$$

This is what `PID_Kinematic::get_orientation_error` implements.

## Step 2: Task-space PID -> twist

Each task-space PID outputs a 3D command (linear and angular). The PID class stores the integral and previous error, so it must be called once per step.

$$
v_{cmd} = K_{p,pos} e_p + K_{i,pos} \int e_p\, dt + K_{d,pos} \frac{d e_p}{dt}
$$

$$
w_{cmd} = K_{p,ori} e_o + K_{i,ori} \int e_o\, dt + K_{d,ori} \frac{d e_o}{dt}
$$

In code this is `pid_pos.all_terms` and `pid_ori.all_terms` from `controllers/PID/PID.cpp`.

## Step 3: Stack into a desired twist

$$
\dot{x}_{des} =
\begin{bmatrix}
v_{cmd} \\
w_{cmd}
\end{bmatrix}
$$

The first 3 entries are linear velocity, the last 3 are angular velocity.

## Step 4: Jacobian pseudoinverse -> desired joint velocity

The twist is mapped to joint space with a damped least squares pseudoinverse:

$$
J^{\dagger} = J^T \left(J J^T + \lambda I\right)^{-1}
$$

$$
\dot{q}_{des} = J^{\dagger} \dot{x}_{des}
$$

The demo uses `lambda = 1e-6` for numerical robustness.

## Step 5: Integrate desired joint position

$$
q_{des} \leftarrow q_{des} + \dot{q}_{des} \Delta t
$$

This is a simple Euler integration in `PID_Kinematic::integrate_joint_des`.

## Step 6: Joint-space PID + gravity compensation

Once you have a desired joint position, run a low-level PID and add gravity compensation:

$$
e_q = q_{des} - q_{arm}
$$

$$
\tau_{pid} = K_{p,q} e_q + K_{i,q} \int e_q\, dt + K_{d,q} \frac{d e_q}{dt}
$$

$$
\tau = \tau_{pid} + g_{arm}
$$

`g_arm` is the arm slice of $g(q)$ from `RobotModel::compute_gravity`, which calls MuJoCo inverse dynamics with $\dot{q} = 0$ and $\ddot{q} = 0$ to isolate gravity.

<!-- ## Implementation map

- Task-space errors and twist stacking: `controllers/PID/kinematic_pid.cpp`
- PID math (P/I/D and anti-windup): `controllers/PID/PID.cpp`
- Jacobian, gravity, and the control loop wiring: `src/main.cpp`
- Gravity computation (inverse dynamics wrapper): `interfaces/robot_model.cpp`

## Practical notes

- Units: positions in meters, orientation error in radians, twists in m/s and rad/s, torques in N*m.
- The PID class is stateful. If you need to reinitialize, call `reset()` or `init()`.
- Damped pseudoinverse avoids singularities but still benefits from reasonable targets and gains. -->
