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

## Boeing 747 - Parameters

<table>
<tr>
<td>


| Aircraft Parameter | Value |
|------------------|------|
| Altitude (ft) | $20000$ |
| Mach | $0.650$ |
| True Airspeed (ft/s) | $673$ |
| Dyn Pressure (lb/ft²) | $287.3$ |
| Weight (lb) | $636636$ |
|Wing Area, S $(ft^2)$|$5500$|
|Wing span, b $(ft)$|$196$|
|Wing chord, $\bar{c}$ $(ft)$|$27.3$|
|C.G $(c \space \bar{x})$|$0.25$|
|Trim AoA (deg)|$2.5$|
|$I_{xx}$ $(slug-ft^2)$|$1.82 \times 10^7$|
|$I_{yy}$ $(slug-ft^2)$|$3.31 \times 10^7$|
|$I_{zz}$ $(slug-ft^2)$|$4.97 \times 10^7$|
|$I_{xz}$ $(slug-ft^2)$|$-4.05 \times 10^7$|
</td>

<td>

|Lateral Derivatives|Value|
|------------------------|----|
|$X_u (1/s)$|$-0.0059$|
|$X_\alpha (ft/s^2)$|$15.9787$|
|$Z_u (1/s)$|$-0.1104$|
|$Z_\alpha (ft/s^2)$|$-353.52$|
|$M_u (1/ft.s)$|$0.000$|
|$M_\alpha (1/s^2)$|$-1.3028$|
|$M_{\dot{\alpha}} (1/s)$|$-0.1057$|
|$M_q (1/s)$|$-0.5417$|
|$X_{\delta_e} (ft/s^2)$|$0.000$|
|$Z_{\delta_e} (ft/s^2)$|$-25.5659$|
|$M_{\delta_e} (1/s^2)$|$-1.6937$|

</td>

<td>

|Longitudinal Derivatives|Value|
|------------------------|----|
|$Y_{\beta} (ft/s^2)$|$-71.9142$|
|$L_{\beta} (1/s^2)$|$-2.7255$|
|$L_p (1/s)$|$-0.8434$|
|$L_r (1/s)$|$0.3224$|
|$N_{\beta} (1/s^2)$|$0.9962$|
|$N_p (1/s)$|$-0.0236$|
|$N_r (1/s)$|$-0.2539$|
|$Y_{\delta_r} (ft/s)^2$|$9.5872$|
|$L_{\delta_r} (1/s^2)$|$0.1363$|
|$N_{\delta_r} (1/s^2)$|$-0.6226$|
|$Y_{\delta_a} (ft/s^2)$|$1.0386$|
|$L_{\delta_a} (1/s^2)$|$0.2214$|
|$N_{\delta_a} (1/s^2)$|$0.0112$|

</td>

</tr>
</table>

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

$$
\dot{x} = Ax + Bu
$$

Aerodynamic data is referenced from "Introduction to Aircraft Flight Mechanics" by Steven L. Morris, David E. Bossert and Wayne F. Hallgren
