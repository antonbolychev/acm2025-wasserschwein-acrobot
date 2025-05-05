import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from matplotlib.animation import FuncAnimation
from pathlib import Path
import tyro
from dataclasses import dataclass
from matplotlib.collections import LineCollection

root_dir = Path(__file__).parent
output_dir = root_dir / "gfx"


class Acrobot:
    def __init__(
        self,
        m1=1.0,
        m2=1.0,
        l1=1.0,
        l2=2.0,
        lc1=0.5,
        lc2=1.0,
        I1=0.083,
        I2=0.33,
        g=9.8,
        b2 = 1.5,
        is_energy_based_only=False,
    ):
        """Initialize Acrobot parameters"""
        self.m1 = m1  # mass of link 1 (kg)
        self.m2 = m2  # mass of link 2 (kg)
        self.l1 = l1  # length of link 1 (m)
        self.l2 = l2  # length of link 2 (m)
        self.lc1 = lc1  # distance to COM of link 1 (m)
        self.lc2 = lc2  # distance to COM of link 2 (m)
        self.I1 = I1  # moment of inertia of link 1 (kg·m²)
        self.I2 = I2  # moment of inertia of link 2 (kg·m²)
        self.g = g  # gravity (m/s²)
        self.b2 = b2 # friction coefficient in joint 2

        # Derived parameters (equation 5 in the paper)
        self.alpha1 = m1 * lc1**2 + m2 * l1**2 + I1
        self.alpha2 = m2 * lc2**2 + I2
        self.alpha3 = m2 * l1 * lc2
        self.beta1 = (m1 * lc1 + m2 * l1) * g
        self.beta2 = m2 * lc2 * g

        # Upright equilibrium energy (equation 12)
        self.Er = self.beta1 + self.beta2

        # Control parameters (tuned based on paper)
        self.kD = 35.8  # must be >35.741 (equation 70)
        self.kP = 61.2  # must be >61.141 (equation 70)
        self.kV = 66.3  # tuned for performance

        self.is_switched = False
        self.is_energy_based_only = is_energy_based_only

    def M(self, q2):
        """Mass matrix (equation 2)"""
        M11 = self.alpha1 + self.alpha2 + 2 * self.alpha3 * np.cos(q2)
        M12 = self.alpha2 + self.alpha3 * np.cos(q2)
        M21 = M12
        M22 = self.alpha2
        return np.array([[M11, M12], [M21, M22]])

    def C(self, q2, dq1, dq2):
        """Coriolis/centripetal terms (equation 3)"""
        H1 = -self.alpha3 * (2 * dq1 * dq2 + dq2**2) * np.sin(q2)
        H2 = self.alpha3 * dq1**2 * np.sin(q2)
        return np.array([H1, H2])

    def G(self, q1, q2):
        """Gravity terms (equation 4)"""
        G1 = self.beta1 * np.cos(q1) + self.beta2 * np.cos(q1 + q2)
        G2 = self.beta2 * np.cos(q1 + q2)
        return np.array([G1, G2])

    def P(self, q1, q2):
        """Potential energy (equation 7)"""
        return self.beta1 * np.sin(q1) + self.beta2 * np.sin(q1 + q2)

    def E(self, q1, q2, dq1, dq2):
        """Total energy (equation 6)"""
        dq = np.array([dq1, dq2])
        kinetic = 0.5 * dq.T @ self.M(q2) @ dq
        potential = self.P(q1, q2)
        return kinetic + potential
    
    def Fr(self, dq2):
        """Friction terms (viscous friction model)"""
        F2 = self.b2 * dq2
        return np.array([0, F2])

    def energy_based_controller(self, t, state):
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

        # Compute numerator and denominator of control law (equation 18)
        numerator = (self.kV * dq2 + self.kP * q2) * Delta + self.kD * (
            M21 * H1_plus_G1 - M11 * H2_plus_G2
        )
        denominator = self.kD * M11 + E_error * Delta

        # Compute control torque
        tau2 = -numerator / denominator

        return tau2

    def pd_controller(self, t, state):
        q1, q2, dq1, dq2 = state
        x = np.array([q1 - np.pi / 2, q2, dq1, dq2])

        F = np.array([-246.481, -98.690, -106.464, -50.138])
        tau2 = -F @ x

        return tau2

    def controller(self, t, state):
        # return self.energy_based_controller(t, state)
        x = np.array([state[0] - np.pi / 2, state[1], state[2], state[3]])
        if self.is_energy_based_only or (
            np.abs(x[0]) + np.abs(x[1]) + 0.1 * np.abs(x[2]) + 0.1 * np.abs(x[3]) > 0.04
            and not self.is_switched
        ):
            return self.energy_based_controller(t, state)
        else:
            self.is_switched = True
            return self.pd_controller(t, state)

    def dynamics(self, t, state):
        """Acrobot dynamics (equation 1)"""
        q1, q2, dq1, dq2 = state

        # Compute control input
        tau2 = self.controller(t, state)
        tau = np.array([0, tau2])  # Acrobot has no actuation at joint 1

        # Compute mass matrix and its inverse
        M_mat = self.M(q2)
        M_inv = np.linalg.inv(M_mat)

        # Compute Coriolis and gravity terms
        C_vec = self.C(q2, dq1, dq2)
        G_vec = self.G(q1, q2)
        F_vec = self.Fr(dq2)

        # Compute accelerations
        ddq = M_inv @ (tau - C_vec - G_vec - F_vec)

        return [dq1, dq2, ddq[0], ddq[1]]

    def simulate(self, t_span, initial_state, t_eval=None):
        """Simulate the Acrobot system"""
        sol = solve_ivp(
            self.dynamics,
            t_span,
            initial_state,
            t_eval=t_eval,
            method="RK45",
            rtol=1e-6,
            atol=1e-6,
        )

        # Calculate tau2 for each time point
        tau2_values = np.array(
            [self.controller(t, sol.y[:, i]) for i, t in enumerate(sol.t)]
        )

        return sol.t, sol.y, tau2_values


def plot_results(t, y, tau2, t_span, output_dir):
    """Plot simulation results with x-axis limited to t_span"""
    plt.figure(figsize=(15, 10), num="Acrobot Simulation Results")

    # Plot angles
    plt.subplot(3, 2, 1)
    plt.plot(t, y[0, :] - np.pi / 2, label="q1 (link 1)")
    plt.plot(t, y[1, :], label="q2 (link 2)")
    plt.xlim(t_span)
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (rad)")
    plt.title("Joint Angles")
    plt.legend()
    plt.grid(True)

    # Plot angular velocities
    plt.subplot(3, 2, 2)
    plt.plot(t, y[2, :], label="dq1")
    plt.plot(t, y[3, :], label="dq2")
    plt.xlim(t_span)
    plt.xlabel("Time (s)")
    plt.ylabel("Angular velocity (rad/s)")
    plt.title("Joint Velocities")
    plt.legend()
    plt.grid(True)

    # Plot phase portrait of q1
    x = y[0, :] - np.pi / 2
    y_phase = y[2, :]
    points = np.array([x, y_phase]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    # Create a LineCollection with a colormap
    norm = plt.Normalize(0.0, t.max())  # Normalize to the index range
    lc = LineCollection(segments, cmap='plasma', norm=norm, linewidth=2)
    lc.set_array(np.linspace(0, t.max(), int(len(segments)//2)))  # Color based on index (time)

    # Plot with gradient and colorbar
    plt.subplot(3, 2, 3)
    ax_phase = plt.gca()
    line = ax_phase.add_collection(lc)

    ax_phase.scatter(x[0],y_phase[0], marker='o', color='green', s=250, label='Start', zorder=6, alpha=0.8)
    ax_phase.scatter(x[-1],y_phase[-1], marker='x', color='red', s=250, label='Finish', zorder=6, alpha=0.8)

    x_range = max(x) - min(x)
    y_range = max(y_phase) - min(y_phase)

    x_padding = x_range * 0.1
    y_padding = y_range * 0.1

    plt.xlim(x.min()-x_padding, x.max()+x_padding)
    plt.ylim(y_phase.min()-y_padding, y_phase.max()+y_padding)
    plt.xlabel("q1 - π/2 (rad)")
    plt.ylabel("dq1 (rad/s)")
    plt.title("Phase Portrait of Link 1")
    plt.grid(True)
    plt.legend()

    # Add colorbar
    cb = plt.colorbar(line, ax=ax_phase)
    cb.set_label("Time (s)")

    # Plot energy
    acrobot = Acrobot()
    E = [acrobot.E(y[0, i], y[1, i], y[2, i], y[3, i]) for i in range(len(t))]
    plt.subplot(3, 2, 4)
    plt.plot(t, E, label="Total Energy")
    plt.axhline(acrobot.Er, color="r", linestyle="--", label="Er (upright energy)")
    plt.xlim(t_span)
    plt.xlabel("Time (s)")
    plt.ylabel("Energy (J)")
    plt.title("System Energy")
    plt.legend()
    plt.grid(True)

    # Plot control torque tau2
    plt.subplot(3, 1, 3)
    plt.plot(t, tau2, label="Control Torque τ₂")
    plt.xlim(t_span)
    plt.xlabel("Time (s)")
    plt.ylabel("Torque (N·m)")
    plt.title("Control Input")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(output_dir / "plots.png")
    print(f"Plots saved to {output_dir}/plots.png")


def animate_acrobot(t, y, t_span):
    """Create animation of acrobot motion"""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, autoscale_on=False, xlim=(-4, 4), ylim=(-4, 4))
    ax.set_aspect("equal")
    ax.grid()
    ax.set_title("Acrobot Animation")

    # Add axis labels and title
    ax.set_xlabel("x (m)", fontsize=12)
    ax.set_ylabel("y (m)", fontsize=12)
    ax.set_title("Acrobot: Swing-up Control", fontsize=14, pad=20)

    # Create acrobot elements with better visualization
    (link1,) = ax.plot([], [], "o-", lw=4, color="royalblue", markersize=8)
    (link2,) = ax.plot([], [], "o-", lw=4, color="crimson", markersize=8)

    # Create proxy artists for the legend if needed
    from matplotlib.lines import Line2D

    legend_elements = [
        Line2D(
            [0], [0], color="royalblue", lw=4, marker="o", markersize=8, label="Link 1"
        ),
        Line2D(
            [0], [0], color="crimson", lw=4, marker="o", markersize=8, label="Link 2"
        ),
    ]

    # Add legend with the elements we want to show
    ax.legend(handles=legend_elements, loc="upper right", fontsize=10)

    time_text = ax.text(0.02, 0.95, "", transform=ax.transAxes)
    energy_text = ax.text(0.02, 0.90, "", transform=ax.transAxes)
    torque_text = ax.text(0.02, 0.85, "", transform=ax.transAxes)
    friction_coeff_text = ax.text(0.02, 0.80, "", transform=ax.transAxes)

    # Calculate link positions
    acrobot = Acrobot()
    x1 = acrobot.l1 * np.sin(y[0, :])
    y1 = -acrobot.l1 * np.cos(y[0, :])
    x2 = x1 + acrobot.l2 * np.sin(y[0, :] + y[1, :])
    y2 = y1 - acrobot.l2 * np.cos(y[0, :] + y[1, :])

    def init():
        link1.set_data([], [])
        link2.set_data([], [])
        time_text.set_text("")
        energy_text.set_text("")
        torque_text.set_text("")
        friction_coeff_text.set_text("")
        return link1, link2, time_text, energy_text, torque_text, friction_coeff_text

    def animate(i):
        # Link 1 (from base to first joint)
        link1.set_data([0, y1[i]], [0, x1[i]])
        # Link 2 (from first joint to end)
        link2.set_data([y1[i], y2[i]], [x1[i], x2[i]])

        time_text.set_text(f"Time = {t[i]:.2f}s")

        # Calculate energy
        E = acrobot.E(y[0, i], y[1, i], y[2, i], y[3, i])
        energy_text.set_text(f"Energy = {E:.2f}J (Er = {acrobot.Er:.2f}J)")

        # Show current torque
        tau2 = acrobot.controller(t[i], y[:, i])
        torque_text.set_text(f"Torque τ₂ = {tau2:.2f}N·m")


        # Show friction_coeff
        friction_coeff_text.set_text(f"Friction coefficient = {acrobot.b2:.2f}")

        return link1, link2, time_text, energy_text, torque_text, friction_coeff_text

    # Choose a reasonable frame rate
    step = max(1, len(t) // 200)
    ani = FuncAnimation(
        fig,
        animate,
        frames=range(0, len(t), step),
        interval=20,
        blit=True,
        init_func=init,
    )

    plt.close()
    return ani


@dataclass
class SimulationParams:
    energy_based_only: bool = False
    output_dir: Path = output_dir

    def __post_init__(self):
        if self.energy_based_only:
            self.output_dir = self.output_dir / "energy_based_only"
        else:
            self.output_dir = self.output_dir / "with_friction"

        self.output_dir.mkdir(parents=True, exist_ok=True)


if __name__ == "__main__":
    params = tyro.cli(SimulationParams)

    # Initialize Acrobot system
    acrobot = Acrobot(
        is_energy_based_only=params.energy_based_only,
    )

    # Initial state (near downward position)
    initial_state = [-1.4, 0.0, 0.0, 0.0]  # [q1, q2, dq1, dq2]

    # Simulation time
    t_span = [0, 30]  # 30 seconds

    # Run simulation with energy-based controller
    t, y, tau2 = acrobot.simulate(t_span, initial_state)

    # Plot results
    plot_results(t, y, tau2, t_span, params.output_dir)

    # Create and display animation
    print("Creating animation...")
    ani = animate_acrobot(t, y, t_span)

    # Save as GIF
    ani.save(params.output_dir / "acrobot.gif", writer="pillow", fps=30)
    print("Animation saved as acrobot.gif")
