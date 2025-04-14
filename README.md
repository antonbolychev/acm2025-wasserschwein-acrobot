# Acrobot

The Acrobot environment is based on Sutton’s work in “Generalization in Reinforcement Learning: Successful Examples Using Sparse Coarse Coding” and Sutton and Barto’s book. The system consists of two links connected linearly to form a chain, with one end of the chain fixed. The joint between the two links is actuated. The goal is to apply torque to the actuated pivot so that the free end of the linear chain moves to a vertical position, starting from an initial downward hanging state.

<!-- ![movie](https://github.com/antonbolychev/acm2025-wasserschwein-acrobot/blob/master/acrobot_animation.gif) -->

![plots](https://github.com/antonbolychev/acm2025-wasserschwein-acrobot/blob/master/plots.jpg)

# Theory

**A two-link planar robot** |
:-------------------------: |
![ ](https://github.com/antonbolychev/acm2025-wasserschwein-acrobot/blob/master/two-link_planner_robot.png)|


1. **Motion equation of a two-link planar robot**:
```math
\begin{equation}
   M(q)\ddot{q} + C(q, \dot{q})\dot{q} + G(q) = \tau , \text{where}\ q = [q_1, q_2]^T
\end{equation}
```

2. **Inertia matrix**:
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

3. **Coriolis and centrifugal terms**:
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

4. **Gravitational terms**:
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

5. **Constants**
```math
\begin{equation}
\begin{align*}
\alpha_1 &= m_1 l_{c1}^2 + m_2 l_1^2 + I_1 \\
\alpha_2 &= m_2 l_{c2}^2 + I_2, \quad \alpha_3 = m_2 l_1 l_{c2} \\
\beta_1 &= (m_1 l_{c1} + m_2 l_1)g, \quad \beta_2 = m_2 l_{c2}g
\end{align*}
\end{equation}
```

6. **Energy of the Acrobot**:
```math
\begin{equation}
   E(q, \dot{q}) = \frac{1}{2} \dot{q}^T M(q) \dot{q} + P(q)
\end{equation}
```

7. **Potential energy**:
```math
\begin{equation}
   P(q) = \beta_1 \sin q_1 + \beta_2 \sin(q_1 + q_2)
\end{equation}
```

8. **Upright equilibrium point**:

```math
q_1 = \frac{\pi}{2} \ (\text{mod} \ 2\pi), \quad q_2 = 0, \quad \dot{q}_1 = 0, \quad \dot{q}_2 = 0
```

9. **Lyapunov function candidate**:

```math
V = \frac{1}{2} (E - E_r)^2 + \frac{1}{2} k_D \dot{q}_2^2 + \frac{1}{2} k_P q_2^2
```

10. **Energy at upright equilibrium**:

```math
E_r = E(q, \dot{q}) \big|_{q_1 = \pi/2, \, q_2 = 0, \, \dot{q} = 0} = \beta_1 + \beta_2
```

11. **Control condition for $`\dot{V} \le 0`$**:

```math
(E - E_r) \tau_2 + k_D \ddot{q}_2 + k_P q_2 = -k_V \dot{q}_2
```

12. **Time derivative of $`V`$**:

```math
\dot{V} = -k_V \dot{q}_2^2 \leq 0
```

13. **Solvability condition for $`\tau_2`$**:

```math
\left( k_D + \frac{(E - E_r) \Delta}{M_{11}} \right) \tau_2 = \frac{(k_V \dot{q}_2 + k_P q_2) \Delta + k_D \big( M_{21}(H_1 + G_1) - M_{11}(H_2 + G_2) \big)}{M_{11}}
```

14. **Positive definiteness of $`\Delta`$**:

```math
\Delta = M_{11} M_{22} - M_{12} M_{21} = \alpha_1 \alpha_2 - \alpha_3^2 \cos^2 q_2 > 0
```

15. **Non-singularity condition**:

```math
k_D + \frac{(E - E_r) \Delta}{M_{11}} \neq 0 \quad \forall t \geq 0
```

16. **Final control law for $`\tau_2`$**:
```math
\tau_2 = \frac{(k_V \dot{q}_2 + k_P q_2) \Delta + k_D \big( M_{21}(H_1 + G_1) - M_{11}(H_2 + G_2) \big)}{k_D M_{11} + (E - E_r) \Delta}
```

# User Guide

## Installation and dependencies:

All necessary dependencies are in file [requirements.txt](https://github.com/antonbolychev/acm2025-wasserschwein-acrobot/requirements.txt)

Make the environment (pipenv or conda): \
`pip`: 
```bash
$ python -m venv your_env
$ source your_env/bin/activate
```
`conda`: 
```bash
$ conda create -n your_env
$ conda activate your_env
```
Then, install the requirements: 
```bash
$ pip install -r requirements.txt
```

To check: 
```bash
$ pip freeze
```
or 
```
$ conda list
```

After creating the environment with all dependencies you need to clone [acm2025-wasserschwein-acrobot](https://github.com/antonbolychev/acm2025-wasserschwein-acrobot) repository:

```bash
$ git clone https://github.com/antonbolychev/acm2025-wasserschwein-acrobot
```

## Run

Run the [acrobot.py](https://github.com/antonbolychev/acm2025-wasserschwein-acrobot/acrobot.py) file:

```bash
cd <PATH_TO_REPO>
python acrobat.py
```

# Authors
* [Egor Miroshnichenko](https://github.com/Chenkomirosh)
* [Anton Bolychev](https://github.com/antonbolychev)
* [Vladislav Sarmatin](https://github.com/VladSarm)
* [Arsenii Shavrin](https://github.com/ArseniiSh)

# References
* [Sutton, R. S. (1996). Generalization in Reinforcement Learning: Successful Examples Using Sparse Coarse Coding. In D. Touretzky, M. C. Mozer, & M. Hasselmo (Eds.), Advances in Neural Information Processing Systems (Vol. 8). MIT Press.](https://proceedings.neurips.cc/paper/1995/file/8f1d43620bc6bb580df6e80b0dc05c48-Paper.pdf
)
* Sutton, R. S., Barto, A. G. (2018 ). Reinforcement Learning: An Introduction. The MIT Press.
<!-- ## Install

If you don't have uv install

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

then run 

```
uv run acrobot.py
``` -->


Here are the formulas (1)-(6) from the PDF rewritten in LaTeX:

1. **Motion equation of a two-link planar robot**:
   \[
   M(q)\ddot{q} + C(q, \dot{q})\dot{q} + G(q) = \tau
   \]

2. **Inertia matrix**:
   \[
   M(q) = \begin{bmatrix} 
   M_{11} & M_{12} \\ 
   M_{21} & M_{22} 
   \end{bmatrix} = \begin{bmatrix} 
   \alpha_1 + \alpha_2 + 2\alpha_3 \cos q_2 & \alpha_2 + \alpha_3 \cos q_2 \\ 
   \alpha_2 + \alpha_3 \cos q_2 & \alpha_2 
   \end{bmatrix}
   \]

3. **Coriolis and centrifugal terms**:
   \[
   C(q, \dot{q})\dot{q} = \begin{bmatrix} 
   H_1 \\ 
   H_2 
   \end{bmatrix} = \alpha_3 \begin{bmatrix} 
   -2\dot{q}_1 \dot{q}_2 - \dot{q}_2^2 \\ 
   \dot{q}_1^2 
   \end{bmatrix} \sin q_2
   \]

4. **Gravitational terms**:
   \[
   G(q) = \begin{bmatrix} 
   G_1 \\ 
   G_2 
   \end{bmatrix} = \begin{bmatrix} 
   \beta_1 \cos q_1 + \beta_2 \cos(q_1 + q_2) \\ 
   \beta_2 \cos(q_1 + q_2) 
   \end{bmatrix}
   \]

5. **Energy of the Acrobot**:
   \[
   E(q, \dot{q}) = \frac{1}{2} \dot{q}^T M(q) \dot{q} + P(q)
   \]

6. **Potential energy**:
   \[
   P(q) = \beta_1 \sin q_1 + \beta_2 \sin(q_1 + q_2)
   \]

Let me know if you'd like any modifications or additional formulas!

