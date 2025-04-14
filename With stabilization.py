import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

class Acrobot:
    def __init__(self, m1=1.0, m2=1.0, l1=1.0, l2=2.0, lc1=0.5, lc2=1.0, 
                 I1=0.083, I2=0.33, g=9.8):
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
        
        # Derived parameters (equation 5 in the paper)
        self.alpha1 = m1*lc1**2 + m2*l1**2 + I1
        self.alpha2 = m2*lc2**2 + I2
        self.alpha3 = m2*l1*lc2
        self.beta1 = (m1*lc1 + m2*l1)*g
        self.beta2 = m2*lc2*g
        
        # Upright equilibrium energy (equation 12)
        self.Er = self.beta1 + self.beta2
        
        # Control parameters (tuned based on paper)
        self.kD = 35.8  # must be >35.741 (equation 70)
        self.kP = 61.2  # must be >61.141 (equation 70)
        self.kV = 66.3  # tuned for performance
        
    def M(self, q2):
        """Mass matrix (equation 2)"""
        M11 = self.alpha1 + self.alpha2 + 2*self.alpha3*np.cos(q2)
        M12 = self.alpha2 + self.alpha3*np.cos(q2)
        M21 = M12
        M22 = self.alpha2
        return np.array([[M11, M12], [M21, M22]])
    
    def C(self, q2, dq1, dq2):
        """Coriolis/centripetal terms (equation 3)"""
        H1 = -self.alpha3*(2*dq1*dq2 + dq2**2)*np.sin(q2)
        H2 = self.alpha3*dq1**2*np.sin(q2)
        return np.array([H1, H2])
    
    def G(self, q1, q2):
        """Gravity terms (equation 4)"""
        G1 = self.beta1*np.cos(q1) + self.beta2*np.cos(q1 + q2)
        G2 = self.beta2*np.cos(q1 + q2)
        return np.array([G1, G2])
    
    def P(self, q1, q2):
        """Potential energy (equation 7)"""
        return self.beta1*np.sin(q1) + self.beta2*np.sin(q1 + q2)
    
    def E(self, q1, q2, dq1, dq2):
        """Total energy (equation 6)"""
        dq = np.array([dq1, dq2])
        kinetic = 0.5 * dq.T @ self.M(q2) @ dq
        potential = self.P(q1, q2)
        return kinetic + potential
    
    def controller(self, t, state):
        """Energy-based controller (equation 18)"""
        q1, q2, dq1, dq2 = state
        
        # Compute energy error
        E = self.E(q1, q2, dq1, dq2)
        E_error = E - self.Er
        
        # Compute mass matrix and its determinant
        M_mat = self.M(q2)
        M11, M12 = M_mat[0,0], M_mat[0,1]
        M21, M22 = M_mat[1,0], M_mat[1,1]
        Delta = M11*M22 - M12*M21  # equation 16
        
        # Compute Coriolis and gravity terms
        C_vec = self.C(q2, dq1, dq2)
        G_vec = self.G(q1, q2)
        H1_plus_G1 = C_vec[0] + G_vec[0]
        H2_plus_G2 = C_vec[1] + G_vec[1]
        
        # Compute numerator and denominator of control law (equation 18)
        numerator = (self.kV*dq2 + self.kP*q2)*Delta + self.kD*(M21*H1_plus_G1 - M11*H2_plus_G2)
        denominator = self.kD*M11 + E_error*Delta
        
        # Compute control torque
        tau2 = - numerator / denominator
        
        return tau2
    
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
        
        # Compute accelerations
        ddq = M_inv @ (tau - C_vec - G_vec)
        
        return [dq1, dq2, ddq[0], ddq[1]]
    
    def simulate(self, t_span, initial_state, t_eval=None):
        """Simulate the Acrobot system"""
        sol = solve_ivp(self.dynamics, t_span, initial_state, t_eval=t_eval, 
                        method='RK45', rtol=1e-6, atol=1e-8)
        return sol
    
    def lqr_controller(self, state):
        """LQR controller for balancing at upright position"""
        q1, q2, dq1, dq2 = state
        x = np.array([q1 - np.pi/2, q2, dq1, dq2])  # state deviation from upright
        
        # LQR gain matrix (from paper)
        F = np.array([-246.481, -98.690, -106.464, -50.138])
        tau2 = -F @ x
        
        return tau2
    
    def hybrid_simulate(self, t_span, initial_state, switch_threshold=0.04):
        """Simulate with swing-up and LQR switching"""
        # First run swing-up controller
        sol = self.simulate(t_span, initial_state)
        
        # Check if we should switch to LQR
        swing_up_time = t_span[-1]
        for i, t in enumerate(sol.t):
            state = sol.y[:,i]
            x = np.array([state[0] - np.pi/2, state[1], state[2], state[3]])
            switch_measure = np.abs(x[0]) + np.abs(x[1]) + 0.1*np.abs(x[2]) + 0.1*np.abs(x[3])
            
            if switch_measure < switch_threshold:
                swing_up_time = t
                break
        
        # If we switched, run LQR controller for remaining time
        if swing_up_time < t_span[-1]:
            # Create new time span
            t_remaining = t_span[-1] - swing_up_time
            t_eval = np.linspace(swing_up_time, t_span[-1], int(100*t_remaining))
            
            # Run LQR simulation
            initial_state_lqr = sol.y[:,np.argmin(np.abs(sol.t - swing_up_time))]
            
            # Need to modify dynamics to use LQR controller
            def lqr_dynamics(t, state):
                q1, q2, dq1, dq2 = state
                tau2 = self.lqr_controller(state)
                tau = np.array([0, tau2])
                
                M_mat = self.M(q2)
                M_inv = np.linalg.inv(M_mat)
                C_vec = self.C(q2, dq1, dq2)
                G_vec = self.G(q1, q2)
                ddq = M_inv @ (tau - C_vec - G_vec)
                
                return [dq1, dq2, ddq[0], ddq[1]]
            
            sol_lqr = solve_ivp(lqr_dynamics, [swing_up_time, t_span[-1]], 
                               initial_state_lqr, t_eval=t_eval, 
                               method='RK45', rtol=1e-6, atol=1e-8)
            
            # Combine solutions
            combined_t = np.concatenate((sol.t[sol.t <= swing_up_time], sol_lqr.t))
            combined_y = np.concatenate((sol.y[:,sol.t <= swing_up_time], sol_lqr.y), axis=1)
            
            return combined_t, combined_y, swing_up_time
        else:
            return sol.t, sol.y, t_span[-1]

def plot_results(t, y, swing_up_time=None):
    """Plot simulation results"""
    plt.figure(figsize=(12, 8))
    
    # Plot angles
    plt.subplot(2, 2, 1)
    plt.plot(t, y[0,:] - np.pi/2, label='q1 (link 1)')
    plt.plot(t, y[1,:], label='q2 (link 2)')
    if swing_up_time is not None:
        plt.axvline(swing_up_time, color='r', linestyle='--', label='Switch to LQR')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (rad)')
    plt.title('Joint Angles')
    plt.legend()
    plt.grid(True)
    
    # Plot angular velocities
    plt.subplot(2, 2, 2)
    plt.plot(t, y[2,:], label='dq1')
    plt.plot(t, y[3,:], label='dq2')
    if swing_up_time is not None:
        plt.axvline(swing_up_time, color='r', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular velocity (rad/s)')
    plt.title('Joint Velocities')
    plt.legend()
    plt.grid(True)
    
    # Plot phase portrait of q1
    plt.subplot(2, 2, 3)
    plt.plot(y[0,:] - np.pi/2, y[2,:])
    plt.xlabel('q1 - π/2 (rad)')
    plt.ylabel('dq1 (rad/s)')
    plt.title('Phase Portrait of Link 1')
    plt.grid(True)
    
    # Plot energy
    acrobot = Acrobot()
    E = [acrobot.E(y[0,i], y[1,i], y[2,i], y[3,i]) for i in range(len(t))]
    plt.subplot(2, 2, 4)
    plt.plot(t, E, label='Total Energy')
    plt.axhline(acrobot.Er, color='r', linestyle='--', label='Er (upright energy)')
    if swing_up_time is not None:
        plt.axvline(swing_up_time, color='r', linestyle='--')
    plt.xlabel('Time (s)')
    plt.ylabel('Energy (J)')
    plt.title('System Energy')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()

# Main simulation
if __name__ == "__main__":
    # Initialize Acrobot system
    acrobot = Acrobot()
    
    # Initial state (near downward position)
    initial_state = [-1.4, 0.0, 0.0, 0.0]  # [q1, q2, dq1, dq2]
    
    # Simulation time
    t_span = [0, 30]  # 10 seconds
    
    # Run hybrid simulation (swing-up + LQR)
    t, y, switch_time = acrobot.hybrid_simulate(t_span, initial_state)
    
    # Plot results
    plot_results(t, y, switch_time)
    
    print(f"Swing-up completed in {switch_time:.2f} seconds")