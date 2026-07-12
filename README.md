# 3-DOF Articulated Robotic Manipulator (3R Serial Arm)

A fully analyzed and hardware-implemented 3-revolute-joint serial manipulator, built as an independent robotics project. The project covers the complete pipeline: mechanical design, kinematic and dynamic derivation, MATLAB simulation/validation, and embedded PID control on real hardware.

<!-- Add a photo or short GIF of the physical arm here -->

## Overview

- **Configuration:** RRR (3 revolute joints), open serial chain
- **Max vertical reach:** 470 mm
- **Actuators:** 3x MG995 metal gear servos (9.4 kg·cm @ 6V)
- **Controller:** ESP32, PWM servo control via the `ledc` peripheral (50 Hz, 14-bit resolution)
- **Design software:** SolidWorks (CAD/assembly), MATLAB Robotics System Toolbox (simulation), Arduino IDE (firmware)

## What's in this repo

| Folder | Contents |
|---|---|
| `cad/` | SolidWorks part/assembly files and STEP exports |
| `matlab/` | Kinematic, Jacobian, and dynamic simulation/validation scripts |
| `firmware/` | ESP32 Arduino code for PWM joint control |
| `docs/` | Full project report (PDF) with complete derivations |
| `media/` | Photos and demo video/GIF of the physical build |

## Technical Highlights

**Kinematics**
- Forward kinematics derived analytically using the Denavit-Hartenberg convention and homogeneous transformation matrices
- Verified against MATLAB's `rigidBodyTree` model, cross-checked at multiple joint-angle test configurations

**Jacobian and Singularity Analysis**
- Full geometric Jacobian derived manually (skew-symmetric matrix method) and independently verified against MATLAB's built-in `geometricJacobian`
- Two singularity conditions identified through determinant analysis: an elbow singularity (fully extended/folded arm) and a workspace-boundary singularity (end-effector on the vertical axis)
- Manipulability index (`w = sqrt(det(Jv * Jv'))`) computed across the reachable workspace to visualize dexterity and flag near-singular regions

**Dynamics**
- Full equations of motion derived via the Euler-Lagrange formulation, including per-link inertia tensors (parallel-axis theorem), the 3x3 mass matrix, Coriolis/centrifugal terms, and the gravity vector
- Mass matrix verified in MATLAB for symmetry and positive definiteness
- Joint torques (tau1, tau2, tau3) computed and cross-checked against the analytical derivation for a test trajectory

**Control**
- Joint-space PID control implemented on ESP32, mapping target joint angles to PWM pulse width (500-2500 microseconds)
- Software joint-limit enforcement and safe home-position initialization on power-up
- `checkTrajectoryPoint.m` used to reject unsafe trajectory waypoints that approach a singular configuration before sending them to hardware

## Validation Approach

Every analytical result in this project (forward kinematics, Jacobian, mass matrix, gravity vector, joint torques) was independently cross-checked against MATLAB's Robotics System Toolbox before being trusted, and the maximum numerical error was printed at each validation step. This was done deliberately: since the hardware implementation depends on this model being correct, each stage had to pass validation before moving to the next.

## Tools Used

SolidWorks - MATLAB (Robotics System Toolbox) - Arduino IDE - ESP32

## Full Report

See [`docs/`](./docs) for the complete project report, including all derivations, experimental workspace/manipulability plots, and the economic/safety/educational analysis.

---

*Sagar Ali - BS Mechatronics and Control Engineering, UET Lahore (Faisalabad Campus)*
