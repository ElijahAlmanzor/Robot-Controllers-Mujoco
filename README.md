# Robot Controllers in MuJoCo (C++)

MuJoCo control demos implemented in C++, exploring kinematic and dynamic control
methods for robotic manipulators.

This repository is intended as a research and experimental platform for
developing, testing, and understanding robot control algorithms in simulation.

---

## Overview

This project provides a lightweight C++ framework for simulating robotic arms
in MuJoCo and experimenting with different control strategies.

The focus is on:
- clarity of control implementations
- direct access to MuJoCo dynamics and kinematics
- clean C++ structure suitable for research and prototyping

At present, the project loads and visualises a Franka Emika Panda robot in MuJoCo.
Control methods will be added incrementally.

---

## Planned Work

The following controllers are planned, though not necessarily in this order:

- Joint space PID kinematic control
- Joint space PID dynamic control
- LQR based dynamic control
- Model Predictive Control (MPC)
- Virtual Model Control
- Cartesian impedance control
- Stochastic impedance control
- Support for different robotic manipulators and arms

---

## Robot Model

The Franka Emika Panda robot and world model are taken from:

https://github.com/JeanElsner/panda_mujoco

The MuJoCo XML and mesh assets are used as provided.
All control logic, simulation code, and build infrastructure in this repository
are authored here.

---

## Build and Run (Windows)

This project uses CMake and vcpkg.

### Requirements
- Windows
- CMake
- Visual Studio toolchain
- MuJoCo
- GLFW (via vcpkg)

### Build and install

From the project root:

```bat
build_install.bat

---

### Run Sim
.\install\bin\control_demos_app.exe

