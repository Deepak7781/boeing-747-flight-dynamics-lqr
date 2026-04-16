# Dynamic Simulation and LQR Control Design for a Boeing 747 Using Linearized Stability Derivatives

---

## Project Overview

This project focuses on modeling the **linear aircraft dynamics** of a Boeing 747 at a specified flight condition and designing an **LQR (Linear Quadratic Regulator)** controller for stability and performance improvement.

The aircraft is modeled using **small disturbance theory**, and the equations of motion are constructed using **dimensional stability derivatives** provided in literature.

---

## Objectives

- Build **longitudinal** and **lateral-directional** state-space models
- Analyze **open-loop dynamic modes**
- Design **LQR controllers** for both axes
- Compare **open-loop vs closed-loop performance**
- Implement simulation in:
  - MATLAB
  - Simulink

---

## Flight Condition Used

Primary simulation is carried out at:

- **Altitude:** 20,000 ft  
- **True Airspeed:** 673 ft/s  
- **Angle of Attack:** 2.5°  
- **Flight Condition:** Steady, level flight  

---

## Theory Used

- Linearized aircraft equations of motion
- Small perturbation theory
- Stability derivatives (dimensional form)
- State-space modeling
- Eigenvalue and modal analysis:
  - Short-period mode
  - Phugoid mode
  - Dutch roll
  - Roll subsidence
  - Spiral mode
- Optimal control (LQR)

---

## Mathematical Model

### Longitudinal States

$$
x_{lon} = [u, \alpha,\ q, \theta]^T
$$

### Lateral-Directional States

$$
x_{lat} = [\beta,\ p,\ r, \phi]^T
$$

### General State-Space Form
