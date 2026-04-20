# Linear Quadratic Regulator (LQR)

## Table of Contents

1. [Introduction](#introduction)
2. [Prerequisites](#prerequisites)
3. [State-Space Representation](#state-space-representation)
4. [Problem Formulation](#problem-formulation)
5. [The Cost Function](#the-cost-function)
6. [Deriving the Optimal Control Law](#deriving-the-optimal-control-law)
7. [The Algebraic Riccati Equation (ARE)](#the-algebraic-riccati-equation-are)
8. [Choosing Q and R Matrices](#choosing-q-and-r-matrices)
9. [Properties of LQR](#properties-of-lqr)
10. [Step-by-Step Design Procedure](#step-by-step-design-procedure)
11. [Worked Example: Inverted Pendulum](#worked-example-inverted-pendulum)
12. [LQR vs PID](#lqr-vs-pid)
13. [Limitations of LQR](#limitations-of-lqr)
14. [Extensions](#extensions)
15. [Summary](#summary)

---

## Introduction

The **Linear Quadratic Regulator (LQR)** is one of the most fundamental and widely used techniques in optimal control theory. It provides a systematic method to design a state-feedback controller for a **linear system** that minimizes a **quadratic cost function** — balancing the trade-off between system performance (how fast states are driven to zero) and control effort (how much energy is used).

LQR answers the question:

> *"Given a linear dynamical system, what is the best control input at every moment such that the system is stabilized while using minimal energy?"*

### Where is LQR Used?

- Aircraft autopilot systems
- Satellite attitude control
- Robotic arm trajectory tracking
- Autonomous vehicle steering
- Industrial process control
- Magnetic levitation systems
- Quadrotor drone stabilization

---

## Prerequisites

Before diving into LQR, you should be familiar with:

- **Linear algebra**: matrices, eigenvalues, matrix transpose, positive definite matrices
- **State-space representation** of dynamical systems
- **Stability theory**: eigenvalue placement, Lyapunov stability
- **Calculus of variations** (helpful, but not strictly required)

---

## State-Space Representation

LQR is designed for systems described in **state-space form**:

```
ẋ(t) = A x(t) + B u(t)
y(t) = C x(t) + D u(t)
```

Where:

| Symbol | Dimension | Description |
|--------|-----------|-------------|
| `x(t)` | n × 1     | State vector |
| `u(t)` | m × 1     | Control input vector |
| `y(t)` | p × 1     | Output vector |
| `A`    | n × n     | System (state) matrix |
| `B`    | n × m     | Input matrix |
| `C`    | p × n     | Output matrix |
| `D`    | p × m     | Feedthrough matrix |

For LQR, we primarily focus on the **state equation**:

```
ẋ(t) = A x(t) + B u(t)
```

### Goal of LQR

Drive the state vector `x(t)` to **zero** (the equilibrium) starting from an initial condition `x(0) = x₀`, while minimizing control effort.

---

## Problem Formulation

### Infinite-Horizon LQR

The standard LQR problem is formulated over an **infinite time horizon**:

**Given** a linear time-invariant (LTI) system:
```
ẋ = Ax + Bu
```

**Find** the control law `u(t)` that minimizes the quadratic cost:

```
J = ∫₀^∞ [ xᵀ(t) Q x(t) + uᵀ(t) R u(t) ] dt
```

**Subject to** the system dynamics.

### Finite-Horizon LQR

A finite-horizon variant also exists:

```
J = xᵀ(T) Sƒ x(T) + ∫₀^T [ xᵀ Q x + uᵀ R u ] dt
```

Where `Sƒ` is a terminal cost matrix and `T` is the final time. The finite-horizon solution yields a **time-varying gain**, whereas the infinite-horizon solution yields a **constant gain** — making the latter more practical for implementation.

---

## The Cost Function

```
J = ∫₀^∞ [ xᵀ Q x + uᵀ R u ] dt
```

The cost function has two terms:

### State Cost: `xᵀ Q x`

- Penalizes **deviation of the states from zero**.
- `Q` is a **symmetric positive semi-definite** matrix (`Q ≥ 0`, i.e., `Q = Qᵀ` and all eigenvalues ≥ 0).
- Large `Q` → controller aggressively drives states to zero → faster response, but higher control effort.

### Control Cost: `uᵀ R u`

- Penalizes **control effort** (energy, actuator usage).
- `R` is a **symmetric positive definite** matrix (`R > 0`, i.e., `R = Rᵀ` and all eigenvalues > 0).
- Large `R` → controller is conservative with inputs → slower response, but less energy used.

### Intuition

Think of it as a dial:

```
← Slow, energy-saving                    Fast, aggressive →
[   Large R   ]  ←————————————→  [   Large Q   ]
```

You tune `Q` and `R` to express your **design requirements**.

---

## Deriving the Optimal Control Law

Using the **Pontryagin Minimum Principle** or **Dynamic Programming (Hamilton-Jacobi-Bellman equation)**, it can be shown that the optimal control law is a **linear state feedback**:

```
u*(t) = -K x(t)
```

Where `K` is the **optimal gain matrix**:

```
K = R⁻¹ Bᵀ P
```

And `P` is a **symmetric positive definite matrix** that satisfies the **Algebraic Riccati Equation (ARE)**.

### The Closed-Loop System

Substituting `u = -Kx` back into the system:

```
ẋ = Ax + B(-Kx) = (A - BK)x
```

The closed-loop dynamics are governed by the matrix `(A - BK)`. LQR guarantees this matrix is **stable** (all eigenvalues have negative real parts), provided:

- `(A, B)` is **controllable** (or at least stabilizable)
- `(A, Q^(1/2))` is **observable** (or at least detectable)

---

## The Algebraic Riccati Equation (ARE)

The core of LQR design is solving the **Algebraic Riccati Equation**:

```
AᵀP + PA - PBR⁻¹BᵀP + Q = 0
```

This is a nonlinear matrix equation in `P`. The solution `P` is:
- **Symmetric**: `P = Pᵀ`
- **Positive definite**: `P > 0`
- **Unique** (under controllability/observability conditions)

### How to Solve the ARE

In practice, the ARE is solved numerically using algorithms such as:

1. **Schur decomposition method** (most common)
2. **Hamiltonian matrix eigenvalue decomposition**
3. **Newton's method** (iterative)

In MATLAB:
```matlab
P = care(A, B, Q, R);   % Continuous Algebraic Riccati Equation
K = R \ (B' * P);       % Optimal gain: K = R⁻¹ Bᵀ P
```

In Python (using `scipy`):
```python
from scipy.linalg import solve_continuous_are
import numpy as np

P = solve_continuous_are(A, B, Q, R)
K = np.linalg.inv(R) @ B.T @ P
```

---

## Choosing Q and R Matrices

Selecting `Q` and `R` is the **designer's primary responsibility** and the most important tuning step in LQR.

### Starting Point: Bryson's Rule

A practical and widely used starting heuristic is **Bryson's Rule**:

```
Q = diag( 1/x₁_max², 1/x₂_max², ..., 1/xₙ_max² )
R = diag( 1/u₁_max², 1/u₂_max², ..., 1/uₘ_max² )
```

Where `xᵢ_max` is the maximum acceptable deviation for state `i`, and `uⱼ_max` is the maximum allowed control input `j`.

This normalizes each state and input relative to their physical limits, giving a dimensionless, balanced cost function.

### Diagonal vs. Full Matrices

- **Diagonal Q and R**: Most common. Each diagonal entry independently penalizes a single state or input. Easy to interpret and tune.
- **Full Q**: Allows penalizing **cross-coupling** between states. Useful when states are physically related.
- **Full R**: Penalizes **correlated inputs**, e.g., in multi-rotor drones where thrust vectors interact.

### Effect of Scaling

| Modification | Effect |
|---|---|
| Increase `Q` (all entries) | Faster state convergence, higher control effort |
| Increase `R` (all entries) | Slower convergence, lower control effort |
| Increase `Q[i,i]` only | State `xᵢ` converges faster |
| Increase `R[j,j]` only | Input `uⱼ` is reduced (conservative) |

### Practical Tips

- Start with **identity matrices** `Q = I`, `R = I`, then adjust.
- Use **Bryson's Rule** as an informed starting point.
- Simulate and iterate — there is no purely analytical rule for "perfect" Q and R.
- If the system is slow to converge, increase `Q`. If actuators are saturating, increase `R`.

---

## Properties of LQR

LQR has several remarkable theoretical guarantees that make it attractive in practice.

### 1. Guaranteed Stability

If `(A, B)` is controllable and `Q ≥ 0` is chosen such that `(A, Q^(1/2))` is observable, then the closed-loop system `(A - BK)` is **asymptotically stable**.

### 2. Gain and Phase Margins (SISO Case)

For single-input systems, LQR provides **guaranteed robustness margins**:

- **Gain Margin**: Infinite (can increase loop gain by any factor without instability)
- **Phase Margin**: At least **60 degrees**

These are significantly better than typical ad-hoc designs, which often target 30–45° phase margin.

> **Note**: These margins apply at the plant input. They do not directly guarantee robustness to sensor noise or unmodeled dynamics.

### 3. Optimality

The LQR solution is **globally optimal** for the specified cost function — no other linear (or nonlinear) controller can achieve a lower value of `J`.

### 4. Separation Principle (with LQE/Kalman Filter)

When combined with a **Kalman Filter** (Linear Quadratic Estimator, LQE) to estimate states from noisy measurements, the resulting **LQG (Linear Quadratic Gaussian)** controller maintains the optimal properties of both components independently. This is the **separation principle**.

---

## Step-by-Step Design Procedure

Here is a concise workflow for designing an LQR controller:

### Step 1: Model the System

Derive or identify the state-space matrices `A`, `B`, `C`, `D` for the linearized system around the desired operating point.

### Step 2: Verify Controllability

Check that the system is controllable (or at least stabilizable):

```
rank(𝒞) = n,  where 𝒞 = [B | AB | A²B | ... | Aⁿ⁻¹B]
```

In MATLAB:
```matlab
rank(ctrb(A, B)) == size(A, 1)
```

In Python:
```python
from numpy.linalg import matrix_rank
from numpy import hstack

def controllability_matrix(A, B):
    n = A.shape[0]
    C = B.copy()
    for i in range(1, n):
        C = hstack([C, np.linalg.matrix_power(A, i) @ B])
    return C

matrix_rank(controllability_matrix(A, B)) == A.shape[0]
```

### Step 3: Choose Q and R

Apply Bryson's Rule or use physical intuition. Start diagonal.

### Step 4: Solve the Algebraic Riccati Equation

Use `care()` (MATLAB) or `solve_continuous_are()` (Python/scipy) to find `P`.

### Step 5: Compute the Optimal Gain

```
K = R⁻¹ Bᵀ P
```

### Step 6: Simulate the Closed-Loop System

Verify performance with `ẋ = (A - BK)x` from various initial conditions. Check:
- Settling time
- Overshoot
- Control input magnitude (watch for saturation)

### Step 7: Iterate on Q and R

Adjust weights based on simulation results until performance requirements are met.

---

## Worked Example: Inverted Pendulum

### System Description

A classic control problem: balance a pendulum upright on a cart.

**States**: `x = [cart_position, cart_velocity, pendulum_angle, angular_velocity]ᵀ`

**Input**: `u = [force on cart]`

### Linearized State-Space (around upright equilibrium)

For a pendulum of mass `m = 0.1 kg`, length `l = 0.3 m`, on a cart of mass `M = 0.5 kg`, `g = 9.81 m/s²`:

```
A = [0,    1,     0,      0  ]
    [0,    0,   -mg/M,    0  ]
    [0,    0,     0,      1  ]
    [0,    0,  g(M+m)/Ml, 0  ]

B = [0, 1/M, 0, -1/(Ml)]ᵀ
```

### MATLAB Implementation

```matlab
m = 0.1; M = 0.5; l = 0.3; g = 9.81;

A = [0, 1,              0,           0;
     0, 0,         -m*g/M,           0;
     0, 0,              0,           1;
     0, 0, g*(M+m)/(M*l),            0];

B = [0; 1/M; 0; -1/(M*l)];

% Bryson's Rule: max deviation = [0.5m, 2m/s, 0.1rad, 1rad/s]
Q = diag([1/0.5^2, 1/2^2, 1/0.1^2, 1/1^2]);

% Max force = 20 N
R = 1/20^2;

% Solve ARE and compute gain
P = care(A, B, Q, R);
K = R \ (B' * P);

% Verify closed-loop eigenvalues
eig(A - B*K)  % All should have negative real parts

% Simulate
sys_cl = ss(A - B*K, B, eye(4), 0);
x0 = [0; 0; 0.05; 0];   % 0.05 rad initial tilt
initial(sys_cl, x0);
```

### Python Implementation

```python
import numpy as np
from scipy.linalg import solve_continuous_are
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

m, M, l, g = 0.1, 0.5, 0.3, 9.81

A = np.array([
    [0, 1, 0, 0],
    [0, 0, -m*g/M, 0],
    [0, 0, 0, 1],
    [0, 0, g*(M+m)/(M*l), 0]
])

B = np.array([[0], [1/M], [0], [-1/(M*l)]])

# Bryson's Rule
Q = np.diag([1/0.5**2, 1/2**2, 1/0.1**2, 1/1**2])
R = np.array([[1/20**2]])

# Solve ARE
P = solve_continuous_are(A, B, Q, R)
K = np.linalg.inv(R) @ B.T @ P

print("Gain K:", K)
print("Closed-loop eigenvalues:", np.linalg.eigvals(A - B @ K))

# Closed-loop simulation
def closed_loop(t, x):
    u = -K @ x
    return (A @ x.reshape(-1,1) + B @ u).flatten()

x0 = [0, 0, 0.05, 0]
sol = solve_ivp(closed_loop, [0, 5], x0, max_step=0.01)

plt.figure(figsize=(10, 6))
labels = ['Cart Position (m)', 'Cart Velocity (m/s)',
          'Pendulum Angle (rad)', 'Angular Velocity (rad/s)']
for i in range(4):
    plt.subplot(2, 2, i+1)
    plt.plot(sol.t, sol.y[i])
    plt.xlabel('Time (s)')
    plt.title(labels[i])
    plt.grid(True)
plt.tight_layout()
plt.show()
```

### Expected Result

The closed-loop eigenvalues should all have **negative real parts**, e.g.:
```
λ ≈ -6.3 + 0j,  -6.3 + 0j,  -2.1 + 1.5j,  -2.1 - 1.5j
```

The pendulum stabilizes from a small initial tilt within a few seconds.

---

## LQR vs PID

| Feature | LQR | PID |
|---|---|---|
| System type | Multivariable (MIMO) | Typically SISO |
| Design basis | Optimal cost minimization | Heuristic tuning |
| State feedback | Full state required | Only error signal |
| Robustness | Guaranteed margins (SISO) | Depends on tuning |
| Ease of tuning | Systematic (Q, R matrices) | Intuitive but iterative |
| Nonlinear systems | Requires linearization | Sometimes works directly |
| Sensor requirement | All states (or observer) | Only controlled variable |
| Industrial adoption | Aerospace, robotics | Very widespread |

**Key takeaway**: LQR is superior for multi-state, multi-input systems where full state information is available and optimal performance is required. PID remains the workhorse for simple, single-loop industrial control.

---

## Limitations of LQR

Despite its elegance, LQR has several important limitations:

### 1. Requires Full State Measurement
LQR assumes **all states are measurable**. In practice, many states are not directly observable (e.g., velocity, internal pressures). This requires combining LQR with a **state observer** (e.g., Kalman Filter), yielding the **LQG controller**.

### 2. Linear Models Only
LQR is derived for **linear systems**. Real systems are often nonlinear. You must linearize around an operating point, and the controller is only valid near that point. For large deviations, performance degrades.

### 3. No Explicit Constraint Handling
LQR does not enforce **hard constraints** on states or inputs (e.g., actuator saturation, position limits). Anti-windup or **Model Predictive Control (MPC)** must be used for constrained problems.

### 4. Robustness is Not Directly Specified
While LQR has guaranteed margins for SISO systems, **MIMO LQR does not guarantee robustness margins** against simultaneous gain perturbations. **Loop Transfer Recovery (LTR)** or robust control methods (H∞) may be needed.

### 5. Perfect Model Assumed
LQR assumes the model `(A, B)` is exact. In reality, model uncertainty exists. **Robust LQR** formulations exist but add complexity.

### 6. Q and R Tuning is Non-Trivial
Translating performance requirements (rise time, overshoot) directly into `Q` and `R` weights is not straightforward and often requires simulation-based iteration.

---

## Extensions

### LQG (Linear Quadratic Gaussian)

Combines LQR with a **Kalman Filter** (LQE) when states are not fully measurable or measurements are noisy:

```
State Estimate:  x̂̇ = Ax̂ + Bu + L(y - Cx̂)
Control Law:     u = -Kx̂
```

`L` is the Kalman gain, designed to minimize estimation error covariance.

### Discrete-Time LQR

For digital implementations (sampled-data systems), the discrete-time version uses:

```
x[k+1] = Aᵈ x[k] + Bᵈ u[k]
J = Σₖ₌₀^∞ [ xᵀ[k] Q x[k] + uᵀ[k] R u[k] ]
```

Solved via the **Discrete Algebraic Riccati Equation (DARE)**:
```matlab
P = dare(Ad, Bd, Q, R);
K = (R + Bd'*P*Bd) \ (Bd'*P*Ad);
```

### Tracking LQR

Modifies the cost function to track a reference trajectory `r(t)` rather than regulate to zero:

```
J = ∫₀^∞ [ (x - r)ᵀ Q (x - r) + uᵀ R u ] dt
```

### Integral LQR

Augments the state with an integrator on the output error to achieve **zero steady-state error** for step references (similar to the integral term in PID).

### Model Predictive Control (MPC)

Generalization of finite-horizon LQR that explicitly handles **state and input constraints** by solving an optimization problem online at each time step. Reduces to LQR in the unconstrained, infinite-horizon case.

---

## Summary

| Concept | Formula / Key Point |
|---|---|
| System | `ẋ = Ax + Bu` |
| Cost Function | `J = ∫(xᵀQx + uᵀRu)dt` |
| Optimal Control | `u* = -Kx` |
| Gain | `K = R⁻¹BᵀP` |
| Riccati Equation | `AᵀP + PA - PBR⁻¹BᵀP + Q = 0` |
| Controllability | Required: `rank([B AB ... Aⁿ⁻¹B]) = n` |
| Phase Margin (SISO) | Guaranteed ≥ 60° |
| Gain Margin (SISO) | Guaranteed = ∞ |

### Design Checklist

- [ ] Derive linearized state-space model `(A, B)`
- [ ] Verify controllability
- [ ] Choose `Q` (state penalty) and `R` (input penalty) using Bryson's Rule
- [ ] Solve the Algebraic Riccati Equation for `P`
- [ ] Compute gain `K = R⁻¹BᵀP`
- [ ] Verify closed-loop stability (all eigenvalues of `A - BK` have negative real parts)
- [ ] Simulate and iterate on `Q` and `R` as needed
- [ ] Consider observer design (Kalman Filter) if full state is not measurable

---

*LQR is a foundational tool in modern control theory. Mastering it opens the door to advanced topics such as LQG, H∞ control, and Model Predictive Control.*
