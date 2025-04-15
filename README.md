# Acrobot

The Acrobot environment is based on Sutton's work in "Generalization in Reinforcement Learning: Successful Examples Using Sparse Coarse Coding" and Sutton and Barto's book. The system consists of two links connected linearly to form a chain, with one end of the chain fixed. The joint between the two links is actuated. The goal is to apply torque to the actuated pivot so that the free end of the linear chain moves to a vertical position, starting from an initial downward hanging state.


<p align="center">
  <img src="gfx/full_stabilization/acrobot.gif" alt="full stablization of acrobot" width="400">
</p>
<p align="center">
  <em>Full stabilization of the Acrobot system using an energy-based controller with PD control transition at the apex</em>
</p>

<p align="center">
  <img src="gfx/energy_based_only/acrobot.gif" alt="full stablization of acrobot" width="400">
</p>
<p align="center">
  <em>Acrobot control using energy-based controller without PD control transition</em>
</p>


# User Guide

## Quick Start

### Prerequisites

If you don't have [uv](https://github.com/astral-sh/uv) (a fast Python package installer and resolver) installed, run:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### Running the Simulation

To run the standard simulation with full stabilization:

```bash
uv run acrobot.py
```

The plots and animation will be saved to [`gfx/full_stabilization/`](./gfx/full_stabilization/).

To run the simulation with only the energy-based controller (without switching to PD control):

```bash
uv run acrobot.py --energy-based-only
```

This alternative simulation output will be saved to [`gfx/energy_based_only/`](./gfx/energy_based_only/).

## Code Overview

* **Single-file implementation**: The entire Acrobot simulation is contained in [`acrobot.py`](./acrobot.py), using tyro for command-line argument parsing.
* **Simulation output**: `Acrobot.simulate()` returns the full trajectory of the system.
* **Visualization & saving**: The `plot_results` and `animation` functions generate and save plots/animations to the `output` folder.

# Theory

## Plant discription

<p align="center">
  <img src="gfx/two-link_planner_robot.png" alt="two link planner robot" width="400">
</p>
<p align="center">
  <em>A two-link planar robot</em>
</p>

Motion equation of a two-link planar robot is
```math
\begin{equation}
   M(q)\ddot{q} + C(q, \dot{q})\dot{q} + G(q) = \tau , \text{where}\ q = [q_1, q_2]^T, \tau = [0, \tau_2]^T
\end{equation}
```
As we worked with Acrobot we put $`\tau_1 = 0`$, $`\tau_2`$ is a control action.
 
**Inertia matrix** have the next form:
```math
\begin{equation}
   M(q) = \begin{bmatrix} 
   M_{11} & M_{12} \\ 
   M_{21} & M_{22} 
   \end{bmatrix} = \begin{bmatrix} 
   \alpha_1 + \alpha_2 + 2\alpha_3 \cos q_2 & \alpha_2 + \alpha_3 \cos q_2 \\ 
   \alpha_2 + \alpha_3 \cos q_2 & \alpha_2 
   \end{bmatrix}
   \end{equation}
```

**Coriolis and centrifugal** terms have the nex form:
```math
\begin{equation}
   C(q, \dot{q})\dot{q} = \begin{bmatrix} 
   H_1 \\ 
   H_2 
   \end{bmatrix} = \alpha_3 \begin{bmatrix} 
   -2\dot{q}_1 \dot{q}_2 - \dot{q}_2^2 \\ 
   \dot{q}_1^2 
   \end{bmatrix} \sin q_2
   \end{equation}
```

**Gravitational** terms have the next form:
```math
\begin{equation}
   G(q) = \begin{bmatrix} 
   G_1 \\ 
   G_2 
   \end{bmatrix} = \begin{bmatrix} 
   \beta_1 \cos q_1 + \beta_2 \cos(q_1 + q_2) \\ 
   \beta_2 \cos(q_1 + q_2) 
   \end{bmatrix}
   \end{equation}
```

For convenience, the following constants are introduced:
```math

\alpha_1 = m_1 l_{c1}^2 + m_2 l_1^2 + I_1 
```
```math
\alpha_2 = m_2 l_{c2}^2 + I_2, \quad \alpha_3 = m_2 l_1 l_{c2} 
```
```math
\beta_1 = (m_1 l_{c1} + m_2 l_1)g, \quad \beta_2 = m_2 l_{c2}g

```

## Energy based controller

The energy of the Acrobot is expressed as
```math
\begin{equation}
   E(q, \dot{q}) = \frac{1}{2} \dot{q}^T M(q) \dot{q} + P(q)
\end{equation}
```

where $`P(q)`$ is the potential energy of the Acrobot and is set as
```math
\begin{equation}
   P(q) = \beta_1 \sin q_1 + \beta_2 \sin(q_1 + q_2)
\end{equation}
```

Observe the following balanced upright state of the Acrobot:

```math
q_1 = \frac{\pi}{2} \ (\text{mod} \ 2\pi), \quad q_2 = 0, \quad \dot{q}_1 = 0, \quad \dot{q}_2 = 0
```
Here, $`q_1`$ (the underactuated cyclic variable) lies in $`\mathbb{S}^1 `$, while $`q_2`$ (the actuated shape variable) is in  $`\mathbb{R}`$. 

We propose the following Lyapunov function candidate::

```math
V = \frac{1}{2} (E - E_r)^2 + \frac{1}{2} k_D \dot{q}_2^2 + \frac{1}{2} k_P q_2^2
```

where $`E_r = \beta_1 + \beta_2`$ represents the Acrobot's energy at the upright equilibrium, and $`k_D`$, $`k_P`$ are positive constants

Compute the total energy at upright equilibrium:

```math
E_r = E(q, \dot{q}) \big|_{q_1 = \pi/2, \, q_2 = 0, \, \dot{q} = 0} = \beta_1 + \beta_2
```

Differentiating  $`V`$ and noting  $`\dot{E} = \dot{q}_2 \tau_2`$ , we obtain:  
```math
\dot{V} = \dot{q}_2 \left( (E - E_r) \tau_2 + k_D \ddot{q}_2 + k_P q_2 \right).  
```
To ensure $`\dot{V} \leq 0`$, we enforce:  
```math
(E - E_r) \tau_2 + k_D \ddot{q}_2 + k_P q_2 = -k_V \dot{q}_2,  
```  
where $`k_V > 0`$. This yields $`\dot{V} = -k_V \dot{q}_2^2 \leq 0`$.  



Solving for $`\tau_2`$ requires ensuring the coefficient of $`\tau_2`$ is non-zero $`\forall t \geq 0`$ . This leads to the critical condition:  
```math
k_D > \max_q f(q), \quad \text{where} \quad f(q) = \frac{(E_r - P(q))\Delta}{M_{11}}.  
```

Now we have solvability condition for $`\tau_2`$:

```math
\left( k_D + \frac{(E - E_r) \Delta}{M_{11}} \right) \tau_2 = \frac{(k_V \dot{q}_2 + k_P q_2) \Delta + k_D \big( M_{21}(H_1 + G_1) - M_{11}(H_2 + G_2) \big)}{M_{11}}
```

where $`\Delta = M_{11} M_{22} - M_{12} M_{21} = \alpha_1 \alpha_2 - \alpha_3^2 \cos^2 q_2 > 0`$

Final control law for $`\tau_2`$ have the next form:
```math
\tau_2 = \frac{(k_V \dot{q}_2 + k_P q_2) \Delta + k_D \big( M_{21}(H_1 + G_1) - M_{11}(H_2 + G_2) \big)}{k_D M_{11} + (E - E_r) \Delta}
```

# Results

## Energy based controller

<p align="center">
  <img src="gfx/energy_based_only/plots.png" alt="full stablization of acrobot" width="800">
</p>

<p align="center">
  <img src="gfx/energy_based_only/acrobot.gif" alt="full stablization of acrobot" width="400">
</p>
<p align="center">
  <em>Acrobot control using energy-based controller without PD control transition</em>
</p>

## Energy based controller with stabilization

<p align="center">
  <img src="gfx/full_stabilization/plots.png" alt="full stablization of acrobot" width="800">
</p>

<p align="center">
  <img src="gfx/full_stabilization/acrobot.gif" alt="full stablization of acrobot" width="400">
</p>
<p align="center">
  <em>Full stabilization of the Acrobot system using an energy-based controller with PD control transition at the apex</em>
</p>

# Authors
* [Egor Miroshnichenko](https://github.com/Chenkomirosh)
* [Anton Bolychev](https://github.com/antonbolychev)
* [Vladislav Sarmatin](https://github.com/VladSarm)
* [Arsenii Shavrin](https://github.com/ArseniiSh)

# References
* [Sutton, R. S. (1996). Generalization in Reinforcement Learning: Successful Examples Using Sparse Coarse Coding. In D. Touretzky, M. C. Mozer, & M. Hasselmo (Eds.), Advances in Neural Information Processing Systems (Vol. 8). MIT Press.](https://proceedings.neurips.cc/paper/1995/file/8f1d43620bc6bb580df6e80b0dc05c48-Paper.pdf
)
* Xin, Xin & Kaneda, M.. (2007). Analysis of the energy‐based swing‐up control of the Acrobot. International Journal of Robust and Nonlinear Control. 17. 1503 - 1524. 10.1002/rnc.1184. 

