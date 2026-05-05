# Eigenvalue Analysis of Boeing 747 Using Linearized Stability Derivatives

---

## Project Overview

This project focuses on the **linear dynamic modeling** and **eigenvalue analysis** of a **Boeing 747** at a specified steady flight condition.

Using the framework of **small disturbance theory**, the aircraft equations of motion are written in **state-space form** using given **dimensional stability derivatives**. The resulting longitudinal and lateral-directional models are then analyzed to identify the classical aircraft dynamic modes and interpret the natural stability characteristics of the aircraft.

---

## Objectives

- Build the **longitudinal** and **lateral-directional** state-space models
- Analyze the **open-loop dynamic modes** using eigenvalues
- Identify:
  - Short-period mode
  - Phugoid mode
  - Dutch roll mode
  - Roll subsidence mode
  - Spiral mode
- Study the natural stability characteristics of the Boeing 747
- Implement the analysis in:
  - MATLAB
  - Simulink

---

## Flight Condition Used

The primary simulation is carried out at the following steady, level flight condition:

- **Altitude:** 20,000 ft
- **Mach number:** 0.650
- **True airspeed:** 673 ft/s
- **Trim angle of attack:** 2.5°
- **Flight condition:** Steady, level flight

---

## Boeing 747 Parameters

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
| Wing Area, $S$ $(ft^2)$ | $5500$ |
| Wing Span, $b$ $(ft)$ | $196$ |
| Mean Aerodynamic Chord, $\bar{c}$ $(ft)$ | $27.3$ |
| C.G. location $(\bar{x}/c)$ | $0.25$ |
| Trim AoA (deg) | $2.5$ |
| $I_{xx}$ $(slug\text{-}ft^2)$ | $1.82 \times 10^7$ |
| $I_{yy}$ $(slug\text{-}ft^2)$ | $3.31 \times 10^7$ |
| $I_{zz}$ $(slug\text{-}ft^2)$ | $4.97 \times 10^7$ |
| $I_{xz}$ $(slug\text{-}ft^2)$ | $-4.05 \times 10^7$ |

</td>

<td>

| Longitudinal Derivatives | Value |
|--------------------------|------:|
| $X_u  (1/s)$ | $-0.0059$ |
| $X_\alpha  (ft/s^2)$ | $15.9787$ |
| $Z_u  (1/s)$ | $-0.1104$ |
| $Z_\alpha  (ft/s^2)$ | $-353.52$ |
| $M_u  (1/ft.s)$ | $0.000$ |
| $M_\alpha  (1/s^2)$ | $-1.3028$ |
| $M_{\dot{\alpha}}  (1/s)$ | $-0.1057$ |
| $M_q  (1/s)$ | $-0.5417$ |
| $X_{\delta_e}  (ft/s^2)$ | $0.000$ |
| $Z_{\delta_e}  (ft/s^2)$ | $-25.5659$ |
| $M_{\delta_e}  (1/s^2)$ | $-1.6937$ |

</td>

<td>

| Lateral Derivatives | Value |
|---------------------|------:|
| $Y_{\beta}  (ft/s^2)$ | $-71.9142$ |
| $L_{\beta} (1/s^2)$ | $-2.7255$ |
| $L_p (1/s)$ | $-0.8434$ |
| $L_r (1/s)$ | $0.3224$ |
| $N_{\beta} (1/s^2)$ | $0.9962$ |
| $N_p (1/s)$ | $-0.0236$ |
| $N_r (1/s)$ | $-0.2539$ |
| $Y_{\delta_r} (ft/s^2)$ | $9.5872$ |
| $L_{\delta_r} (1/s^2)$ | $0.1363$ |
| $N_{\delta_r} (1/s^2)$ | $-0.6226$ |
| $Y_{\delta_a} (ft/s^2)$ | $1.0386$ |
| $L_{\delta_a} (1/s^2)$ | $0.2214$ |
| $N_{\delta_a} (1/s^2)$ | $0.0112$ |

</td>

</tr>
</table>

---

## Theory Used

The following theory forms the basis of the project:

- Linearized aircraft equations of motion
- Small perturbation theory
- Dimensional stability derivatives
- State-space modeling
- Eigenvalue and modal analysis

The classical dynamic modes studied are:

### Longitudinal Modes
- Short-period mode
- Phugoid mode

### Lateral-Directional Modes
- Dutch roll mode
- Roll subsidence mode
- Spiral mode

---

## Governing Linear State-Space Model

For small perturbations about a steady trim condition, the aircraft dynamics are written as

$$
\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u}
$$

where:

- $\mathbf{x}$ is the state vector
- $\mathbf{u}$ is the control input vector
- $A$ is the system matrix
- $B$ is the control matrix

For this project, the main interest is in the **system matrix $A$**, because its eigenvalues determine the natural dynamic behavior of the aircraft.

The eigenvalues are obtained from

$$
\det(\lambda I - A) = 0
$$

and are used to assess:

- stability
- oscillatory behavior
- damping
- mode speed
- natural dynamic characteristics

---

## Longitudinal State-Space Model

The longitudinal state vector is chosen as

$$
\mathbf{x}_{lon} =
\begin{bmatrix}
u & \alpha & q & \theta
\end{bmatrix}^T
$$

where:

- $u$ = perturbation in forward velocity
- $\alpha$ = perturbation in angle of attack
- $q$ = pitch rate
- $\theta$ = pitch angle

The control input is taken as elevator deflection:

$$
\mathbf{u}_{lon} =
\begin{bmatrix}
\delta_e
\end{bmatrix}
$$

### Standard Longitudinal Equations

A commonly used linearized longitudinal model in this state form is

$$
\dot{u} = X_u u + X_\alpha \alpha - g \cos\theta_0 \, \theta + X_{\delta_e}\delta_e
$$

$$
\dot{\alpha} = \frac{Z_u}{U_0}u + \frac{Z_\alpha}{U_0}\alpha + \left(1 + \frac{Z_q}{U_0}\right)q - \frac{g \sin\theta_0}{U_0}\theta + \frac{Z_{\delta_e}}{U_0}\delta_e
$$

$$
\dot{q} = M_u u + M_\alpha \alpha + M_q q + M_{\delta_e}\delta_e
$$

$$
\dot{\theta} = q
$$

For the present model, these equations are arranged in the compact form

$$
\dot{\mathbf{x}}_{lon} = A_{lon}\mathbf{x}_{lon} + B_{lon}\delta_e
$$

with

$$
A_{lon} =
\begin{bmatrix}
X_u & X_\alpha & 0 & -g\cos\theta_0 \\
\dfrac{Z_u}{U_0} & \dfrac{Z_\alpha}{U_0} & 1 & -\dfrac{g\sin\theta_0}{U_0} \\
M_u & M_\alpha & M_q & 0 \\
0 & 0 & 1 & 0
\end{bmatrix}
$$

and

$$
B_{lon} =
\begin{bmatrix}
X_{\delta_e} \\
\dfrac{Z_{\delta_e}}{U_0} \\
M_{\delta_e} \\
0
\end{bmatrix}
$$

This model is used to analyze the longitudinal motion and identify the short-period and phugoid modes through the eigenvalues of $A_{lon}$.

---

## Lateral-Directional State-Space Model

The lateral-directional state vector is chosen as

$$
\mathbf{x}_{lat} =
\begin{bmatrix}
\beta & p & r & \phi
\end{bmatrix}^T
$$

where:

- $\beta$ = sideslip angle
- $p$ = roll rate
- $r$ = yaw rate
- $\phi$ = bank angle

The control input vector is taken as

$$
\mathbf{u}_{lat} =
\begin{bmatrix}
\delta_a \\
\delta_r
\end{bmatrix}
$$

where:

- $\delta_a$ = aileron deflection
- $\delta_r$ = rudder deflection

The linearized lateral-directional model is written as

$$
\dot{\mathbf{x}}_{lat} = A_{lat}\mathbf{x}_{lat} + B_{lat}\mathbf{u}_{lat}
$$

A standard form of the lateral-directional equations is

$$
\dot{\beta} = \frac{Y_\beta}{U_0}\beta + \frac{Y_p}{U_0}p + \left(\frac{Y_r}{U_0} - 1\right)r + \frac{g\cos\theta_0}{U_0}\phi + \frac{Y_{\delta_a}}{U_0}\delta_a + \frac{Y_{\delta_r}}{U_0}\delta_r
$$

$$
\dot{p} = L_\beta \beta + L_p p + L_r r + L_{\delta_a}\delta_a + L_{\delta_r}\delta_r
$$

$$
\dot{r} = N_\beta \beta + N_p p + N_r r + N_{\delta_a}\delta_a + N_{\delta_r}\delta_r
$$

$$
\dot{\phi} = p + \tan\theta_0\,r
$$

Thus, the state and input matrices can be written as

$$
A_{lat} =
\begin{bmatrix}
\dfrac{Y_\beta}{U_0} & \dfrac{Y_p}{U_0} & \left(\dfrac{Y_r}{U_0} - 1\right) & \dfrac{g\cos\theta_0}{U_0} \\
L_\beta & L_p & L_r & 0 \\
N_\beta & N_p & N_r & 0 \\
0 & 1 & \tan\theta_0 & 0
\end{bmatrix}
$$

and

$$
B_{lat} =
\begin{bmatrix}
\dfrac{Y_{\delta_a}}{U_0} & \dfrac{Y_{\delta_r}}{U_0} \\
L_{\delta_a} & L_{\delta_r} \\
N_{\delta_a} & N_{\delta_r} \\
0 & 0
\end{bmatrix}
$$

This model is used to analyze the lateral-directional motion and identify the Dutch roll, roll subsidence, and spiral modes through the eigenvalues of $A_{lat}$.

---

## Eigenvalue Analysis

The eigenvalues of the state matrices are obtained from

$$
\det(\lambda I - A) = 0
$$

The eigenvalues indicate whether the motion is stable or unstable, oscillatory or non-oscillatory, and fast or slow.

---

## Longitudinal Eigenvalues

The longitudinal eigenvalues are

$$
\lambda_{lon} =
\begin{cases}
-0.5873 + 1.1149i \\
-0.5873 - 1.1149i \\
-0.0020 + 0.0658i \\
-0.0020 - 0.0658i
\end{cases}
$$

### Interpretation of Longitudinal Modes

The first complex conjugate pair,

$$
-0.5873 \pm 1.1149i
$$

represents the **short-period mode**.

Characteristics of this mode:

- oscillatory
- relatively fast
- reasonably well damped
- dominated mainly by angle of attack and pitch rate variations

The second complex conjugate pair,

$$
-0.0020 \pm 0.0658i
$$

represents the **phugoid mode**.

Characteristics of this mode:

- oscillatory
- very slow
- very lightly damped
- dominated by exchange between kinetic energy and potential energy

Since both conjugate pairs have negative real parts, the longitudinal motion is **stable**.

---

## Lateral-Directional Eigenvalues

The lateral-directional eigenvalues are

$$
\lambda_{lat} =
\begin{cases}
-0.1183 + 1.0373i \\
-0.1183 - 1.0373i \\
-0.9505 \\
-0.0171
\end{cases}
$$

### Interpretation of Lateral-Directional Modes

The complex conjugate pair,

$$
-0.1183 \pm 1.0373i
$$

represents the **Dutch roll mode**.

Characteristics of this mode:

- oscillatory
- stable
- coupled yaw-roll motion

The real eigenvalue,

$$
-0.9505
$$

represents the **roll subsidence mode**.

Characteristics of this mode:

- non-oscillatory
- fast
- strongly stable
- associated mainly with roll-rate decay

The real eigenvalue,

$$
-0.0171
$$

represents the **spiral mode**.

Characteristics of this mode:

- non-oscillatory
- very slow
- stable
- associated with long-term bank and heading tendency

Since all lateral-directional eigenvalues have negative real parts, the lateral-directional motion is also **stable**.

---

## Stability Interpretation

From the eigenvalues, the following conclusions can be drawn:

### Longitudinal Stability
- The short-period mode is stable and reasonably well damped
- The phugoid mode is stable but very lightly damped
- Overall longitudinal motion is stable

### Lateral-Directional Stability
- The Dutch roll mode is stable
- The roll subsidence mode is stable and fast
- The spiral mode is stable but slow
- Overall lateral-directional motion is stable

Thus, at the chosen flight condition, the Boeing 747 exhibits **stable open-loop dynamic behavior** in both longitudinal and lateral-directional motion.

---

## Importance of Eigenvalue Analysis

Eigenvalue analysis is important because it gives direct insight into the aircraft's natural behavior without requiring extensive time-domain simulations first.

It helps determine:

- whether the aircraft is dynamically stable
- which modes are oscillatory
- which modes are fast or slow
- how strongly each mode is damped
- whether the aircraft has any tendency toward divergence

Thus, eigenvalue analysis is one of the most fundamental tools in aircraft stability and control studies.

---
