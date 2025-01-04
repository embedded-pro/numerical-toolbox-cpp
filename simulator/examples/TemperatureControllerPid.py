from simulator.controllers.Pid import Pid, PidTunings, PidLimits
from simulator.examples.TemperaturePlant import TemperaturePlant
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple, Optional

def run_simulation(pid_controller: Pid, 
                  plant: TemperaturePlant,
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
    time_points = np.arange(simulation_steps) * plant.sample_time
    setpoints = np.zeros(simulation_steps)
    temperatures = np.zeros(simulation_steps)
    control_signals = np.zeros(simulation_steps)
    
    # Initialize first temperature
    temperatures[0] = plant.prev_output
    
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
    # Initialize controller
    tunnings = PidTunings(kp=1.0, ki=0.1, kd=0.35)
    limits = PidLimits(min=-100.0, max=100.0)
    
    controller = Pid(
        tunnings=tunnings,
        limits=limits
    )
    
    # Initialize plant
    plant = TemperaturePlant(sample_time=0.1)
    
    # Define setpoint schedule: (time_step, temperature)
    setpoint_schedule = [
        (0, 20),    # Start at room temperature
        (100, 60),  # Heat to 60°C
        (300, 40),  # Cool to 40°C
        (500, 50)   # Final setpoint 50°C
    ]
    
    # Run simulation
    results = run_simulation(
        pid_controller=controller,
        plant=plant,
        setpoint_schedule=setpoint_schedule,
        simulation_steps=700
    )
    
    # Plot results
    plot_results(results)
