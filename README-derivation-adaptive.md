# Mathematical Derivations and Analysis: Acrobot Control System

This document provides detailed mathematical derivations and analysis for the Acrobot control system. It serves as a comprehensive reference to understand the theoretical foundations of the control strategies implemented in the repository.

## Table of Contents
- [Plant Description](#plant-description)
- [Adaptive Control: Detailed Derivation](#adaptive-control-detailed-derivation)
  - [System Energy](#system-energy)
  - [Lyapunov Function Candidate](#lyapunov-function-candidate)
  - [Control Law Derivation](#control-law-derivation)
  - [Solvability Condition Analysis](#solvability-condition-analysis)
- [Stability Analysis](#stability-analysis)
- [Controller Implementation Details](#controller-implementation-details)
  - [Adaptive Controller](#energy-based-controller)
  - [PD Controller for Stabilization](#pd-controller-for-stabilization)
  - [Controller Switching Strategy](#controller-switching-strategy)
- [References](#references)

## Plant Description

The Acrobot is a two-link planar robot with a single actuator at the joint of the two links. It serves as a highly simplified model of a human gymnast on a high bar, where the underactuated first joint models the gymnast's hands on the bar, and the actuated second joint models the gymnast's hips. In new version we introduce a viscous friction int he second joint.

### System Dynamics

The motion equation of a two-link planar robot is given by:

```math
M(q)\ddot{q} + C(q, \dot{q})\dot{q} + G(q) +
\begin{bmatrix}
0 \\
b_2 \dot{q}_2
\end{bmatrix}
=
\begin{bmatrix}
0 \\
\tau_2
\end{bmatrix}
```

where $q = [q_1, q_2]^T$ represents the joint angles, and $\tau = [0, \tau_2]^T$ represents the torques applied to joints (with $\tau_1 = 0$ for the Acrobot as the first joint is not actuated). $b_2$ is a coefficient of viscous friction.

The **inertia matrix** $M(q)$ is defined as:

```math
M(q) = \begin{bmatrix} 
M_{11} & M_{12} \\ 
M_{21} & M_{22} 
\end{bmatrix} = \begin{bmatrix} 
\alpha_1 + \alpha_2 + 2\alpha_3 \cos q_2 & \alpha_2 + \alpha_3 \cos q_2 \\ 
\alpha_2 + \alpha_3 \cos q_2 & \alpha_2 
\end{bmatrix}
```

The **Coriolis and centrifugal** terms are given by:

```math
C(q, \dot{q})\dot{q} = \begin{bmatrix} 
H_1 \\ 
H_2 
\end{bmatrix} = \alpha_3 \begin{bmatrix} 
-2\dot{q}_1 \dot{q}_2 - \dot{q}_2^2 \\ 
\dot{q}_1^2 
\end{bmatrix} \sin q_2
```

The **gravitational** terms are defined as:

```math
G(q) = \begin{bmatrix} 
G_1 \\ 
G_2 
\end{bmatrix} = \begin{bmatrix} 
\beta_1 \cos q_1 + \beta_2 \cos(q_1 + q_2) \\ 
\beta_2 \cos(q_1 + q_2) 
\end{bmatrix}
```

The following constant parameters are introduced for convenience:

```math
\alpha_1 = m_1 l_{c1}^2 + m_2 l_1^2 + I_1
```
```math
\alpha_2 = m_2 l_{c2}^2 + I_2
```
```math
\alpha_3 = m_2 l_1 l_{c2}
```
```math
\beta_1 = (m_1 l_{c1} + m_2 l_1)g
```
```math
\beta_2 = m_2 l_{c2}g
```

where:
- $m_i$ is the mass of link $i$
- $l_i$ is the length of link $i$
- $l_{ci}$ is the distance to the center of mass of link $i$
- $I_i$ is the moment of inertia of link $i$
- $g$ is the acceleration due to gravity

## Adaptive Control: Detailed Derivation

### System Energy

The total energy of the Acrobot system is expressed as:

```math
E(q, \dot{q}) = \frac{1}{2} \dot{q}^T M(q) \dot{q} + P(q)
```

where $P(q)$ is the potential energy defined as:

```math
P(q) = \beta_1 \sin q_1 + \beta_2 \sin(q_1 + q_2)
```

Note that we define the potential energy with respect to the horizontal configuration, not the usual downward position. This convention aligns the upright equilibrium with the goal state.

### Lyapunov Function Candidate

We consider the balanced upright equilibrium state of the Acrobot:

```math
q_1 = \frac{\pi}{2} \quad(\text{mod} \, 2\pi), \quad q_2 = 0, \quad \dot{q}_1 = 0, \quad \dot{q}_2 = 0
```

The energy at this upright equilibrium position is:

```math
E_r = E(q, \dot{q}) \big|_{q_1 = \pi/2, \, q_2 = 0, \, \dot{q} = 0} = \beta_1 + \beta_2
```

We define the following Lyapunov function candidate:

```math
V = \frac{1}{2} (E - E_r)^2 + \frac{1}{2} k_D \dot{q}_2^2 + \frac{1}{2} k_P q_2^2 + \frac{(\hat b_2-b_2)^2}{2\gamma}
```

where $k_D$,$k_P$ and $\gamma$ are positive constants. This Lyapunov function has four components:
1. Energy error squared: $(E - E_r)^2$
2. Velocity damping term: $k_D \dot{q}_2^2$
3. Position regulation term: $k_P q_2^2$
4. Adaptation term: $\frac{(\hat b_2-b_2)^2}{2\gamma}$

### Control Law Derivation

Taking the time derivative of the Lyapunov function candidate along the system trajectory:

```math
\dot{V} = (E - E_r)\dot{E} + k_D \dot{q}_2 \ddot{q}_2 + k_P q_2 \dot{q}_2 + \frac{\dot{\hat b_2} - b_2}{\gamma} \dot{\hat{b_2}}
```

Since $\dot{E} = \dot{q}^T \tau = \dot{q}_2 \tau_2 - b_2 \dot q^2_2$ (as $\tau_1 = 0$), we have:

```math
\dot{V} = \dot{q}_2 \left( (E - E_r) (\tau_2 - \hat{b_2}\dot{q_2}) + k_D \ddot{q}_2 + k_P q_2 \right) + (\hat b - b_2)(\dot{q_2^2}(E-E_r)+ \frac{\dot{\hat b}}{\gamma})
```

To ensure $\dot{V} \leq 0$:

1. Let $\hat{\dot b} = -\gamma \dot q_2^2(E-E_r)$, then $\hat b(t) = \int_0^t{-\gamma \dot q_2^2(E-E_r)dt}$;
2. Due to the fact, that first part should be less or equal zero: $(E - E_r) (\tau_2 - \hat{b_2}\dot{q_2}) + k_D \ddot{q}_2 + k_P q_2 = -k_V \dot{q}_2$

where $k_V > 0$ is a damping coefficient. This makes:

```math
\dot{V} = -k_V \dot{q}_2^2 \leq 0
```

Now, to solve for the control input $\tau_2$, we need to express $\ddot{q}_2$ in terms of system variables and $\tau_2$.

From the system dynamics, the second row gives us:

```math
M_{21}\ddot{q}_1 + M_{22}\ddot{q}_2 + H_2 + G_2 + \hat b_2\dot q_2= \tau_2
```

From the first row (noting $\tau_1 = 0$):

```math
M_{11}\ddot{q}_1 + M_{12}\ddot{q}_2 + H_1 + G_1 = 0
```

Solving for $\ddot{q}_1$ from the first equation:

```math
\ddot{q}_1 = -\frac{M_{12}\ddot{q}_2 + H_1 + G_1}{M_{11}}
```

Substituting into the second equation:

```math
M_{21}\left(-\frac{M_{12}\ddot{q}_2 + H_1 + G_1}{M_{11}}\right) + M_{22}\ddot{q}_2 + H_2 + G_2 + \hat b_2\dot q_2= \tau_2
```

Rearranging:

```math
\left(M_{22} - \frac{M_{21}M_{12}}{M_{11}}\right)\ddot{q}_2 = \tau_2 + \frac{M_{21}(H_1 + G_1)}{M_{11}} - H_2 - G_2 - \hat b_2\dot q_2
```

Noting that $M_{22} - \frac{M_{21}M_{12}}{M_{11}} = \frac{\Delta}{M_{11}}$ where $\Delta = M_{11}M_{22} - M_{12}M_{21}$ is the determinant of the inertia matrix, we get:

```math
\ddot{q}_2 = \frac{M_{11}}{\Delta}\tau_2 + \frac{M_{21}(H_1 + G_1) - M_{11}(H_2 + G_2 + \hat b_2\dot q_2)}{\Delta}
```

Now, substituting this expression for $\ddot{q}_2$ into our desired control law:

```math
(E - E_r) \tau_2 + k_D \left(\frac{M_{11}}{\Delta}\tau_2 + \frac{M_{21}(H_1 + G_1) - M_{11}(H_2 + G_2 + \hat b_2\dot q_2)}{\Delta}\right) + k_P q_2 = -k_V \dot{q}_2
```

Rearranging to solve for $\tau_2$:

```math
\left(E - E_r + \frac{k_D M_{11}}{\Delta}\right) \tau_2 = -k_V \dot{q}_2 - k_P q_2 - \frac{k_D[M_{21}(H_1 + G_1) - M_{11}(H_2 + G_2 + \hat b_2\dot q_2)]}{\Delta}
```

Multiplying both sides by $\frac{\Delta}{M_{11}}$:

```math
\left(k_D + \frac{(E - E_r)\Delta}{M_{11}}\right) \tau_2 = -\frac{(k_V \dot{q}_2 + k_P q_2)\Delta + k_D[M_{21}(H_1 + G_1) - M_{11}(H_2 + G_2 + \hat b_2\dot q_2)]}{M_{11}}
```

Thus, the control law becomes:

```math
\tau_2 = -\frac{(k_V \dot{q}_2 + k_P q_2)\Delta + k_D[M_{21}(H_1 + G_1) - M_{11}(H_2 + G_2 + \hat b_2\dot q_2)]}{k_D M_{11} + (E - E_r)\Delta}
```

### Solvability Condition Analysis

For the control law to be well-defined, we need the denominator to be non-zero:

```math
k_D M_{11} + (E - E_r)\Delta \neq 0
```

This is guaranteed if:

```math
k_D > \max_q f(q)
```

where:

```math
f(q) = \frac{(E_r - P(q))\Delta}{M_{11}}
```

This condition ensures that the denominator never becomes zero, avoiding singularities in the control law.

The proof that this condition is both necessary and sufficient is as follows:

1. **Sufficiency**: If $k_D > \max_q f(q)$, then $k_D M_{11} > (P(q) - E_r)\Delta$ for all $q$. Since $E(q,\dot{q}) \geq P(q)$, we have $(E - E_r)\Delta \geq (P(q) - E_r)\Delta$, thus $k_D M_{11} + (E - E_r)\Delta > 0$ for all states.

2. **Necessity**: If $k_D \leq \max_q f(q)$, then there exists some configuration $\dot q$ where $k_D \leq f(\dot q)$. At this configuration, if we set velocities such that the kinetic energy exactly equals $f(\dot q) - k_D$, then the denominator becomes zero, creating a singularity.

The maximum value of $f(q)$ can be determined through analysis of its critical points. Since $\Delta/M_{11}$ is only a function of $q_2$, the critical points occur when $\partial P(q)/\partial q_1 = 0$, which leads to:

```math
\beta_1 \cos q_1 + \beta_2 \cos(q_1 + q_2) = 0
```

This allows us to express $P(q)$ at critical points as:

```math
P(q) = \pm\sqrt{\beta_1^2 + \beta_2^2 + 2\beta_1\beta_2\cos q_2}
```

Taking the negative value (which maximizes $f(q)$), we get the condition:

```math
k_D > \max_{q_2 \in [0,2\pi]} \frac{(\sqrt{\beta_1^2 + \beta_2^2 + 2\beta_1\beta_2\cos q_2} + E_r)\Delta(q_2)}{M_{11}(q_2)}
```

The right-hand side can be computed numerically, and for standard Acrobot parameters, it's approximately 35.741.

## Stability Analysis

Using LaSalle's invariance principle, we can analyze the behavior of the closed-loop system. Since $\dot{V} \leq 0$, the system trajectories converge to the largest invariant set within:

```math
S = \{(q, \dot{q}, \hat b_2) | \dot{V} = 0\} = \{(q, \dot{q}, \hat b_2) | \dot{q}_2 = 0\}
```

Within this set, the system dynamics enforce $\ddot{q}_2 = 0$ as well, which means:

```math
M_{21}\ddot{q}_1 + H_2 + G_2 + \hat b_2 \dot q_2 = \tau_2
```

The convergence behavior depends on the system parameters and control gains. A thorough analysis shows that all trajectories, except for a set of Lebesgue measure zero, eventually converge to arbitrarily small neighborhoods of the upright equilibrium.

## Controller Implementation Details

### Adaptive Controller

The implementation of the energy-based controller directly follows the derived equation:

```python
def adaptive_controller(self, t, state):
    """Energy-based controller (equation 18)"""
    q1, q2, dq1, dq2 = state

    # Compute energy error
    E = self.E(q1, q2, dq1, dq2)
    E_error = E - self.Er

    # Compute mass matrix and its determinant
    M_mat = self.M(q2)
    M11, M12 = M_mat[0, 0], M_mat[0, 1]
    M21, M22 = M_mat[1, 0], M_mat[1, 1]
    Delta = M11 * M22 - M12 * M21  # equation 16

    # Compute Coriolis and gravity terms
    C_vec = self.C(q2, dq1, dq2)
    G_vec = self.G(q1, q2)
    H1_plus_G1 = C_vec[0] + G_vec[0]
    H2_plus_G2 = C_vec[1] + G_vec[1]

    # Compute adaptive control term
    dt = t - self.t_prev
    self.b2_hat.append(self.b2_hat[-1] + (-self.gamma * E_error * dq2**2) * dt)
    self.t_prev = t
    # Compute numerator and denominator of control law (equation 18)
    numerator = (self.kV * dq2 + self.kP * q2) * Delta + self.kD * (
        M21 * H1_plus_G1 - M11 * H2_plus_G2 - M11 * self.b2_hat[-1] * dq2
    )
    denominator = self.kD * M11 + E_error * Delta

    # Compute control torque
    tau2 = -numerator / denominator

    return tau2
```

The control parameters satisfy the following constraints:
- $k_D > 35.741$ (solvability condition)
- $k_P, k_V > 0$ (for Lyapunov stability)

In the implementation, we use:
- $k_D = 35.8$
- $k_P = 61.2$
- $k_V = 66.3$
- $\gamma = 0.00512$ (tuned parameter)

### PD Controller for Stabilization

Once the Acrobot is close to the upright position, a linear PD controller is used for balancing:

```python
def pd_controller(self, t, state):
    q1, q2, dq1, dq2 = state
    x = np.array([q1 - np.pi / 2, q2, dq1, dq2])
    
    F = np.array([-246.481, -98.690, -106.464, -50.138])
    tau2 = -F @ x
    
    return tau2
```

The gains for the PD controller are determined through linearization of the system around the upright equilibrium point. The specific values were obtained through pole placement to ensure asymptotic stability.

### Controller Switching Strategy

A switching strategy is employed to transition from the energy-based controller to the PD controller when the system is close enough to the upright position:

```python
def controller(self, t, state):
    x = np.array([state[0] - np.pi / 2, state[1], state[2], state[3]])
    if self.is_energy_based_only or (
        np.abs(x[0]) + np.abs(x[1]) + 0.1 * np.abs(x[2]) + 0.1 * np.abs(x[3]) > 0.06
        and not self.is_switched
    ):
        return self.energy_based_controller(t, state)
    else:
        self.is_switched = True
        return self.pd_controller(t, state)
```

The switching condition uses a weighted sum of state deviations from the upright equilibrium. When this sum falls below a threshold (0.06), the controller switches to the PD controller for stabilization.

The option `is_energy_based_only` allows running the simulation with only the energy-based controller, without switching to the PD controller, for comparison purposes.

## References

1. Xin, X., & Kaneda, M. (2007). Analysis of the energy-based swing-up control of the Acrobot. International Journal of Robust and Nonlinear Control, 17(16), 1503-1524.

2. Fantoni, I., Lozano, R., & Spong, M. W. (2000). Energy based control of the Pendubot. IEEE Transactions on Automatic Control, 45(4), 725-729.

3. Kolesnichenko, O., & Shiriaev, A. S. (2002). Partial stabilization of underactuated Euler-Lagrange systems via a class of feedback transformations. Systems & Control Letters, 45(2), 121-132.

4. Sutton, R. S. (1996). Generalization in Reinforcement Learning: Successful Examples Using Sparse Coarse Coding. In Advances in Neural Information Processing Systems, Vol. 8. MIT Press.
