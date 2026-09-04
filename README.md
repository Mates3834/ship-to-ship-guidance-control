# Ship-to-Ship Guidance, Path Planning & Adaptive Control

An integrated autonomous guidance and control framework combining **path planning, trajectory generation, guidance, nonlinear 6-DoF dynamics, control, and state estimation** for ship-to-ship engagement scenarios.

The project investigates how different control and estimation architectures perform within a unified closed-loop system.

---

## System Architecture

The overall architecture integrates:

**Path Planning → Trajectory Smoothing → Guidance → Control → 6-DoF Dynamics → State Estimation → Feedback**

The framework consists of:

- A* based path planning
- Minimum-Snap and SPC-based trajectory smoothing
- Line-of-Sight (LOS) based reference generation
- Nonlinear 6-DoF dynamic modeling
- LQR, LQI and Modified MRAC controllers
- Luenberger Observer
- Kalman Filter
- Monte Carlo based performance evaluation

---

## Path Planning

A grid-based **A*** algorithm is used to generate a collision-free discrete path between the initial position and the target.

The generated path is subsequently refined to obtain a continuous and dynamically trackable reference trajectory.

### Methods

- A* Search
- Collision-Free Path Generation
- Waypoint Generation
- Minimum-Snap Trajectory Smoothing
- SPC-Based Trajectory Smoothing
- Reference Trajectory Generation

---

## Guidance

The guidance layer generates reference attitude commands based on **Line-of-Sight (LOS) geometry**.

The generated reference commands are continuously updated according to the relative geometry between the vehicle and the target.

---

## Nonlinear 6-DoF Model

The guidance and control algorithms are evaluated using a nonlinear **six-degree-of-freedom (6-DoF)** dynamic model.

The model includes:

- Translational dynamics
- Rotational dynamics
- Position and attitude states
- Body / inertial coordinate transformations
- Actuator dynamics and constraints
- Closed-loop guidance and control interaction

---

## Control Systems

Three control architectures are evaluated.

### Linear Quadratic Regulator — LQR

Optimal state-feedback control designed to balance tracking performance and control effort.

### Linear Quadratic Integral — LQI

Integral action is introduced to improve reference tracking and reduce steady-state error.

### Modified Model Reference Adaptive Control — MRAC

An adaptive control architecture is investigated to improve tracking performance under modeling uncertainties and changing engagement conditions.

A robust modification is incorporated into the adaptive mechanism to improve stability and parameter behavior.

---

## State Estimation

Two state-estimation approaches are investigated:

### Luenberger Observer

Used for model-based state reconstruction within the closed-loop control architecture.

### Kalman Filter

Used for target-state estimation in the presence of measurement uncertainty and noise.

---

## Simulation & Performance Analysis

The complete architecture is evaluated through multiple engagement scenarios and Monte Carlo simulations.

Performance evaluation considers:

- Trajectory tracking
- Attitude tracking
- RMS tracking error
- Control effort
- State-estimation performance
- Target interception performance
- Robustness under changing initial conditions

The performances of **LQR, LQI and Modified MRAC** are compared under the same simulation framework.

---

## Technologies

`MATLAB` `Simulink`

`Path Planning` `A*` `Minimum-Snap`

`Guidance Navigation & Control`

`6-DoF Dynamics`

`LQR` `LQI` `MRAC`

`Kalman Filter` `Luenberger Observer`

`Monte Carlo Simulation`

---

## Research Areas

- Autonomous Systems
- Guidance, Navigation & Control
- Path Planning
- Trajectory Optimization
- Adaptive Control
- Optimal Control
- Nonlinear Dynamic Systems
- State Estimation
- Maritime Autonomous Systems

---

## Repository Structure

```text
ship-to-ship-guidance-control/
│
├── README.md
├── src/
│   ├── path_planning/
│   ├── guidance/
│   ├── control/
│   └── estimation/
│
├── models/
│   └── simulink/
│
├── results/
│   ├── trajectories/
│   ├── tracking/
│   └── estimation/
│
└── docs/
