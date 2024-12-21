import numpy as np
import matplotlib.pyplot as plt

class ClarkeParkTransformation:
    @staticmethod
    def abc_to_alpha_beta(ia, ib, ic):
        """Transform three-phase currents to alpha-beta frame"""
        alpha = (2/3) * (ia - 0.5*ib - 0.5*ic)
        beta = (2/3) * (np.sqrt(3)/2 * ib - np.sqrt(3)/2 * ic)
        return alpha, beta
    
    @staticmethod
    def alpha_beta_to_dq(alpha, beta, theta):
        """Transform alpha-beta to dq frame"""
        d = alpha * np.cos(theta) + beta * np.sin(theta)
        q = -alpha * np.sin(theta) + beta * np.cos(theta)
        return d, q
    
    @staticmethod
    def dq_to_alpha_beta(d, q, theta):
        """Transform dq to alpha-beta frame"""
        alpha = d * np.cos(theta) - q * np.sin(theta)
        beta = d * np.sin(theta) + q * np.cos(theta)
        return alpha, beta
    
    @staticmethod
    def alpha_beta_to_abc(alpha, beta):
        """Transform alpha-beta to three-phase"""
        a = alpha
        b = -0.5*alpha + (np.sqrt(3)/2)*beta
        c = -0.5*alpha - (np.sqrt(3)/2)*beta
        return a, b, c

class PIController:
    def __init__(self, kp, ki, dt):
        self.kp = kp
        self.ki = ki
        self.dt = dt
        self.integral = 0
        self.prev_error = 0
        
    def compute(self, setpoint, feedback):
        error = setpoint - feedback
        self.integral += error * self.dt
        output = self.kp * error + self.ki * self.integral
        self.prev_error = error
        return output, error

class SpaceVectorPWM:
    def __init__(self, switching_freq):
        self.switching_freq = switching_freq
        
    def generate_pwm(self, time_points, v_alpha, v_beta):
        # Generate carrier wave
        carrier = np.mod(time_points * self.switching_freq, 1.0)
        carrier = 2 * np.where(carrier < 0.5, carrier, 1 - carrier)
        
        # Convert alpha-beta to three-phase modulation signals
        ma, mb, mc = ClarkeParkTransformation.alpha_beta_to_abc(v_alpha, v_beta)
        
        # Normalize and shift to 0-1 range
        ma = 0.5 + 0.5 * ma / np.max(np.abs([ma, mb, mc]))
        mb = 0.5 + 0.5 * mb / np.max(np.abs([ma, mb, mc]))
        mc = 0.5 + 0.5 * mc / np.max(np.abs([ma, mb, mc]))
        
        # Generate PWM signals
        pwm_a = np.where(ma > carrier, 1, 0)
        pwm_b = np.where(mb > carrier, 1, 0)
        pwm_c = np.where(mc > carrier, 1, 0)
        
        return pwm_a, pwm_b, pwm_c, ma, mb, mc, carrier

class PMSMMotor:
    def __init__(self, Rs, Ld, Lq, poles, psi_f, J, dt):
        self.Rs = Rs      # Stator resistance
        self.Ld = Ld      # d-axis inductance
        self.Lq = Lq      # q-axis inductance
        self.poles = poles  # Number of pole pairs
        self.psi_f = psi_f  # Permanent magnet flux
        self.J = J         # Moment of inertia
        self.dt = dt       # Time step
        
        # State variables
        self.id = 0
        self.iq = 0
        self.omega_m = 0   # Mechanical speed
        self.theta_e = 0   # Electrical angle
        
    def step(self, vd, vq, load_torque=0):
        # Motor equations in dq frame
        did_dt = (vd - self.Rs*self.id + self.omega_m*self.poles*self.Lq*self.iq) / self.Ld
        diq_dt = (vq - self.Rs*self.iq - self.omega_m*self.poles*(self.Ld*self.id + self.psi_f)) / self.Lq
        
        # Update currents
        self.id += did_dt * self.dt
        self.iq += diq_dt * self.dt
        
        # Electromagnetic torque
        Te = 1.5 * self.poles * (self.psi_f * self.iq + (self.Ld - self.Lq) * self.id * self.iq)
        
        # Mechanical equation
        domega_dt = (Te - load_torque) / self.J
        self.omega_m += domega_dt * self.dt
        
        # Update electrical angle
        self.theta_e += self.omega_m * self.poles * self.dt
        self.theta_e = np.mod(self.theta_e, 2*np.pi)
        
        return self.id, self.iq, self.omega_m, self.theta_e

class ControlSystem:
    def __init__(self, motor_params, control_params, simulation_params):
        # Initialize components
        self.motor = PMSMMotor(**motor_params)
        self.svpwm = SpaceVectorPWM(simulation_params['switching_freq'])
        self.transformations = ClarkeParkTransformation()
        
        # Initialize controllers
        self.d_controller = PIController(**control_params['d_control'])
        self.q_controller = PIController(**control_params['q_control'])
        
        self.dt = simulation_params['dt']
        
    def simulate(self, t_start, t_end, id_ref, iq_ref):
        time_points = np.arange(t_start, t_end, self.dt)
        n_points = len(time_points)
        
        # Arrays to store results
        results = {
            'id': np.zeros(n_points),
            'iq': np.zeros(n_points),
            'ia': np.zeros(n_points),
            'ib': np.zeros(n_points),
            'ic': np.zeros(n_points),
            'pwm_a': np.zeros(n_points),
            'pwm_b': np.zeros(n_points),
            'pwm_c': np.zeros(n_points),
            'vd_ctrl': np.zeros(n_points),
            'vq_ctrl': np.zeros(n_points),
            'theta': np.zeros(n_points)
        }
        
        for i, t in enumerate(time_points):
            # Current feedback
            id_fb, iq_fb = self.motor.id, self.motor.iq
            
            # PI control
            vd, ed = self.d_controller.compute(id_ref, id_fb)
            vq, eq = self.q_controller.compute(iq_ref, iq_fb)
            
            # Convert to alpha-beta
            v_alpha, v_beta = self.transformations.dq_to_alpha_beta(vd, vq, self.motor.theta_e)
            
            # Generate PWM
            pwm_a, pwm_b, pwm_c, ma, mb, mc, carrier = self.svpwm.generate_pwm(
                np.array([t]), v_alpha, v_beta)
            
            # Apply voltage and simulate motor
            id_fb, iq_fb, omega, theta = self.motor.step(vd, vq)
            
            # Transform currents to abc frame for measurement
            ia, ib, ic = self.transformations.alpha_beta_to_abc(
                *self.transformations.dq_to_alpha_beta(id_fb, iq_fb, theta))
            
            # Store results
            results['id'][i] = id_fb
            results['iq'][i] = iq_fb
            results['ia'][i] = ia
            results['ib'][i] = ib
            results['ic'][i] = ic
            results['pwm_a'][i] = pwm_a[0]
            results['pwm_b'][i] = pwm_b[0]
            results['pwm_c'][i] = pwm_c[0]
            results['vd_ctrl'][i] = vd
            results['vq_ctrl'][i] = vq
            results['theta'][i] = theta
            
        return time_points, results

def plot_results(time, results):
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 15))
    
    # Plot three-phase currents
    ax1.plot(time, results['ia'], label='ia')
    ax1.plot(time, results['ib'], label='ib')
    ax1.plot(time, results['ic'], label='ic')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Current (A)')
    ax1.set_title('Three-Phase Currents')
    ax1.legend()
    ax1.grid(True)
    
    # Plot control signals
    ax2.plot(time, results['vd_ctrl'], label='vd')
    ax2.plot(time, results['vq_ctrl'], label='vq')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Control Voltage')
    ax2.set_title('Control Signals')
    ax2.legend()
    ax2.grid(True)
    
    # Plot PWM signals (zoomed)
    zoom_start = 0
    zoom_end = 0.001
    mask = (time >= zoom_start) & (time <= zoom_end)
    
    ax3.plot(time[mask], results['pwm_a'][mask], label='PWM A')
    ax3.plot(time[mask], results['pwm_b'][mask], label='PWM B')
    ax3.plot(time[mask], results['pwm_c'][mask], label='PWM C')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('PWM Signal')
    ax3.set_title('PWM Signals (1ms zoom)')
    ax3.legend()
    ax3.grid(True)
    
    plt.tight_layout()
    plt.show()

# Example usage
if __name__ == "__main__":
    # Motor parameters
    motor_params = {
        'Rs': 0.5,    # Stator resistance
        'Ld': 0.008,  # d-axis inductance
        'Lq': 0.008,  # q-axis inductance
        'poles': 4,   # Number of pole pairs
        'psi_f': 0.171,  # Permanent magnet flux
        'J': 0.0027,    # Moment of inertia
        'dt': 1e-5      # Time step
    }
    
    # Control parameters
    control_params = {
        'd_control': {'kp': 50, 'ki': 500, 'dt': 1e-5},
        'q_control': {'kp': 50, 'ki': 500, 'dt': 1e-5}
    }
    
    # Simulation parameters
    simulation_params = {
        'dt': 1e-5,
        'switching_freq': 10000  # 10kHz switching frequency
    }
    
    # Create control system
    system = ControlSystem(motor_params, control_params, simulation_params)
    
    # Run simulation
    time, results = system.simulate(0, 0.1, id_ref=0, iq_ref=10)
    
    # Plot results
    plot_results(time, results)
