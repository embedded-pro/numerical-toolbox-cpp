from simulator.controllers.Pid import Pid, PidTunings, PidLimits
from simulator.examples.TemperaturePlant import TemperaturePlant
from simulator.math.QNumber import QNumber
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple, Optional, TypeVar
import logging

T = TypeVar('T', float, QNumber)

class ScaledPid(Pid):
    def __init__(self, 
                scale_factor: float,
                tunnings: PidTunings[T],
                limits: PidLimits[T],
                auto_mode: bool = True):
        super().__init__(tunnings=tunnings, limits=limits, auto_mode=auto_mode)
        
        if scale_factor <= 0:
            raise ValueError("Scale factor must be positive")
        
        self.scale_factor = scale_factor
        self.squared_scale_factor = self.scale_factor * self.scale_factor

    def set_point(self, value: T):
        super().set_point(value / self.scale_factor)
    
    def process(self, measured_process_variable: T) -> T:
        return  super().process(measured_process_variable / self.scale_factor) * self.squared_scale_factor

def run_simulation(pid_controller: Pid, 
                  plant: TemperaturePlant,
                  sample_time: float,
                  setpoint_schedule: List[Tuple[int, float]],
                  simulation_steps: int) -> dict:
    """
    Run simulation with given controller and plant.
    
    Args:
        pid_controller: PID controller instance
        plant: Temperature plant instance
        setpoint_schedule: List of (step, temperature) tuples for setpoint changes
        simulation_steps: Number of simulation steps
        
    Returns:
        dict: Simulation results
    """
    # Initialize arrays for storing results
    time_points = np.arange(simulation_steps) * sample_time
    setpoints = np.zeros(simulation_steps)
    temperatures = np.zeros(simulation_steps)
    control_signals = np.zeros(simulation_steps)
    
    # Initialize first temperature
    temperatures[0] = plant.last_output()
    
    # Create setpoint array from schedule
    current_setpoint = setpoint_schedule[0][1]
    current_schedule_idx = 0
    
    for step in range(simulation_steps):
        # Update setpoint if needed
        if (current_schedule_idx < len(setpoint_schedule) - 1 and 
            step >= setpoint_schedule[current_schedule_idx + 1][0]):
            current_schedule_idx += 1
            current_setpoint = setpoint_schedule[current_schedule_idx][1]
        
        setpoints[step] = current_setpoint
        
        pid_controller.set_point(current_setpoint)
        control_signal = pid_controller.process(temperatures[step])
        control_signals[step] = control_signal
        
        if step < simulation_steps - 1:
            temperatures[step + 1] = plant.update(control_signal)
    
    return {
        'time': time_points,
        'setpoint': setpoints,
        'temperature': temperatures,
        'control_signal': control_signals
    }

def plot_results(results: dict, title: str = 'Temperature Control Simulation'):
    """
    Plot simulation results.
    
    Args:
        results: Dictionary containing simulation results
        title: Plot title
    """
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    
    # Plot temperature and setpoint
    ax1.plot(results['time'], results['temperature'], 'b-', label='Temperature')
    ax1.plot(results['time'], results['setpoint'], 'r--', label='Setpoint')
    ax1.grid(True)
    ax1.set_ylabel('Temperature (°C)')
    ax1.legend()
    ax1.set_title(title)
    
    # Plot control signal
    ax2.plot(results['time'], results['control_signal'], 'g-', label='Control Signal')
    ax2.grid(True)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Control Signal')
    ax2.legend()
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)

    scale = 100.0

    tunnings = PidTunings(kp=1.0 / scale, ki=0.1 / scale, kd=0.35 / scale)
    limits = PidLimits(min=-100.0 / scale, max=100.0 / scale)
    
    controller = ScaledPid(
        scale_factor=scale,
        tunnings=tunnings,
        limits=limits
    )
    
    plant = TemperaturePlant(
        sample_time=0.1
    )
    
    # Define setpoint schedule: (time_step, temperature)
    setpoint_schedule = [
        (0, 20),    # Start at room temperature
        (100, 60),  # Heat to 60°C
        (300, 40),  # Cool to 40°C
        (500, 50)   # Final setpoint 50°C
    ]
    
    results = run_simulation(
        pid_controller=controller,
        plant=plant,
        sample_time=plant.sample_time,
        setpoint_schedule=setpoint_schedule,
        simulation_steps=700
    )
    
    plot_results(results)
