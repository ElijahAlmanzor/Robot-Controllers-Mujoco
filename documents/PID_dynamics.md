# Inverse-Dynamics PID Control (Computed Torque)

## Overview

This note documents the mathematics used by the inverse-dynamics task-space PID controller implemented in this demo.  
Unlike the kinematic PID controller, this scheme explicitly compensates for robot dynamics using the inertia, Coriolis, and gravity terms.

The controller structure is:

1. Task-space PID on end-effector position.
2. Convert desired Cartesian acceleration to desired joint acceleration using Jacobian kinematics.
3. Use computed torque control to cancel nonlinear dynamics.
4. Apply torques to the plant, optionally compensating external wrenches.

This is a model-based controller. When the model is accurate, the closed-loop task-space dynamics reduce to a set of decoupled second-order systems.

---

## Notation (matches variable names in code)

Symbol meanings:

`q`, `qdot`, `qddot`  
$q$, $\dot q$, $\ddot q$ — joint position, velocity, acceleration

`x`  
$x$ — end-effector position in Cartesian space

`xref`  
$x_{\mathrm{ref}}$ — target Cartesian position

`e`  
$e$ — Cartesian position error ($x_{\mathrm{ref}} - x$)

`xdot`, `xddot`  
$\dot x$, $\ddot x$ — end-effector velocity and acceleration

`J(q)`  
$J(q)$ — end-effector Jacobian

`Jdot(q, qdot)`  
$\dot J(q, \dot q)$ — time derivative of the Jacobian

`M(q)`  
$M(q)$ — joint-space inertia matrix

`C(q, qdot)`  
$C(q, \dot q)$ — Coriolis and centripetal matrix

`G(q)`  
$G(q)$ — gravity torque vector

`Fext`  
$F_{\mathrm{ext}}$ — external Cartesian wrench applied at the end effector

`tau`  
$\tau$ — commanded joint torques

`dt`  
$dt$ — control timestep

---

## Step 1: Task-space error

The Cartesian position error is computed directly:

\[
e = x_{\mathrm{ref}} - x
\]

Velocity error assumes a zero target velocity:

\[
\dot e = -\dot x
\]

---

## Step 2: Task-space PID to desired Cartesian acceleration

A PID controller is applied directly in task space to generate a desired Cartesian acceleration:

\[
\ddot x_{\mathrm{des}} = K_p e + K_i \int e\,dt + K_d \dot e
\]

This differs fundamentally from the kinematic PID, which outputs a velocity.  
Here, the output is an acceleration that is explicitly enforced through inverse dynamics.

---

## Step 3: Acceleration kinematics

The exact relationship between joint and Cartesian acceleration is:

\[
\ddot x = J(q)\ddot q + \dot J(q, \dot q)\dot q
\]

The $\dot J \dot q$ term captures acceleration induced purely by configuration-dependent changes in the Jacobian.

---

## Step 4: Desired joint acceleration

To enforce $\ddot x = \ddot x_{\mathrm{des}}$, solve:

\[
J(q)\ddot q_{\mathrm{des}} = \ddot x_{\mathrm{des}} - \dot J(q, \dot q)\dot q
\]

Using the Moore–Penrose pseudoinverse:

\[
\ddot q_{\mathrm{des}} = J^\dagger\left(\ddot x_{\mathrm{des}} - \dot J \dot q\right)
\]

This corresponds directly to the implementation:

\[
\ddot q_{\mathrm{des}} = \mathrm{pinv}(J)\left(\ddot x_{\mathrm{des}} - \dot J\,\dot\theta\right)
\]

At this stage, the controller has produced a desired joint acceleration, not a torque.

---

## Step 5: Robot dynamics model

The joint-space dynamics are:

\[
M(q)\ddot q + C(q, \dot q)\dot q + G(q) = \tau + J(q)^{\mathsf{T}} F_{\mathrm{ext}}
\]

This model is used both for control and for simulation.

---

## Step 6: Computed torque control (dynamic compensation)

The control objective is:

\[
\ddot q \approx \ddot q_{\mathrm{des}}
\]

Choose the commanded torque as:

\[
\tau = M(q)\ddot q_{\mathrm{des}} + C(q, \dot q)\dot q + G(q)
\]

This cancels the nonlinear dynamics exactly (for a perfect model) and shapes the closed-loop behaviour via $\ddot q_{\mathrm{des}}$ (i.e., `qddot_des`).

---

## Step 7: External wrench compensation (optional)

If a known external wrench $F_{\mathrm{ext}}$ acts on the end effector, it enters the dynamics as $J(q)^{\mathsf{T}} F_{\mathrm{ext}}$.

To cancel its effect in control:

\[
\tau = \tau - J(q)^{\mathsf{T}} F_{\mathrm{ext}}
\]

If this term is omitted, steady-state task-space error will generally persist unless integral action compensates for it.

---

## Step 8: Plant integration

The simulated plant integrates the full dynamics:

\[
\ddot q = M(q)^{-1}\left(\tau + J(q)^{\mathsf{T}} F_{\mathrm{ext}} - C(q, \dot q)\dot q - G(q)\right)
\]

This matches the inverse-dynamics control law and verifies correct compensation.

---

## Key differences vs kinematic PID

Kinematic PID:
- Controls velocity  
- No dynamics
- Gravity added heuristically  

Inverse-dynamics PID:
- Controls acceleration  
- Explicit inertia shaping  
- Gravity canceled analytically  
- Model-heavy

The inverse-dynamics controller reduces the nonlinear robot to a set of linear second-order systems in task space, assuming **accurate models**.

---

