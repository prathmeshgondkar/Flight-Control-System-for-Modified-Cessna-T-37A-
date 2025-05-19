# ✈️ Flight Control System Simulation - Cessna T-37A (Modified)

This repository contains MATLAB scripts and a Simulink model that simulate the dynamics and control of a modified Cessna T-37A aircraft under steady cruise and engine-out conditions. The objective is to model, linearize, and simulate flight stability using state-space representation and a PID control system.

## 📁 Contents
- `Trim_analysis Final.m`: MATLAB script to perform trim condition analysis using both cruise and OEI (one engine inoperable) assumptions.
- `AC_Model_Jacobian 3.m`: Script that defines 12-state nonlinear equations of motion (EOM), linearizes the system, derives A/B matrices, and implements a basic PID controller.
- `AircraftControlSystem 3.slx`: Simulink model integrating the linearized state-space system in a closed-loop feedback control setup.
- `MECH 6091 Final Project Report.pdf`: Complete technical report documenting methodology, analysis, and results.

## ⚙️ Features
- Nonlinear model with full dynamics (u, v, w, p, q, r, φ, θ, ψ)
- Linearization around cruise condition (α = 2°, β = 0)
- PID-based closed-loop control with MIMO considerations
- Simulink simulation with wind disturbance
- Visualization of 3D aircraft trajectory and velocity response

## 🚧 Limitations
- Due to the complexity of MIMO PID tuning (144 parameters), the model struggles with disturbance rejection.
- Stability is demonstrated in steady state, but not under strong dynamic changes.

## 📌 Future Improvements
- Implement LQR or MPC for better control performance
- Explore adaptive control or decoupling strategies for MIMO systems

## 👨‍💻 Team
- Prathmesh Deepak Gondkar
- Phillip Arab
- Radhouen Khmiri

Supervised by: **Dr. Jonathan Liscouët**  
Course: MECH 6091 – Flight Control Systems

## 📬 Contact
Please contact me on [LinkedIn](www.linkedin.com/in/prathmeshgondkar) if you have any questions or need collaboration.

---

> “Control is the art of making things behave the way we want them to — even when nature disagrees.”
