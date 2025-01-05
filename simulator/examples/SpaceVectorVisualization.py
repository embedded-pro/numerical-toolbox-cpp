import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

class SpaceVectorVisualization:
    def __init__(self, update_interval_ms=50):
        """
        Initialize the Space Vector visualization.
        
        Args:
            update_interval_ms: Animation update interval in milliseconds
        """
        self.radius = 1.0
        self.update_interval = update_interval_ms
        
        self.fig = plt.figure(figsize=(15, 6))
        self.ax_vector = self.fig.add_subplot(121)
        self.ax_duties = self.fig.add_subplot(122)
        
        self._setup_vector_plot()
        self._setup_duties_plot()
        
        self.vector_line = None
        self.current_angle_line = None
        self.duty_points = None
        
    def _setup_vector_plot(self):
        """Setup the vector diagram subplot."""
        hex_vertices = np.array([
            [self.radius * np.cos(i * np.pi/3), self.radius * np.sin(i * np.pi/3)]
            for i in range(7)
        ])
        self.ax_vector.plot(hex_vertices[:, 0], hex_vertices[:, 1], 'k--', alpha=0.5)
        
        for i in range(6):
            angle = i * np.pi/3 + np.pi/6
            x_text = 1.2 * self.radius * np.cos(angle)
            y_text = 1.2 * self.radius * np.sin(angle)
            self.ax_vector.text(x_text, y_text, f'S{i+1}', ha='center', va='center')
        
        self.ax_vector.set_aspect('equal')
        self.ax_vector.set_xlim(-1.5, 1.5)
        self.ax_vector.set_ylim(-1.5, 1.5)
        self.ax_vector.grid(True, alpha=0.3)
        self.ax_vector.set_xlabel('α-axis')
        self.ax_vector.set_ylabel('β-axis')
        self.ax_vector.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        self.ax_vector.axvline(x=0, color='k', linestyle='-', alpha=0.3)
        
    def _setup_duties_plot(self):
        """Setup the duty cycles subplot."""
        self.ax_duties.set_xlim(0, 360)
        self.ax_duties.set_ylim(-0.1, 1.1)
        self.ax_duties.grid(True, alpha=0.3)
        self.ax_duties.set_xlabel('Angle (degrees)')
        self.ax_duties.set_ylabel('Duty Cycle')
        
        angles = np.linspace(0, 360, 360)
        duties = self._calculate_duties(angles)
        
        self.ax_duties.plot(angles, duties[0], 'r-', label='Phase A', alpha=0.5)
        self.ax_duties.plot(angles, duties[1], 'g-', label='Phase B', alpha=0.5)
        self.ax_duties.plot(angles, duties[2], 'b-', label='Phase C', alpha=0.5)
        self.ax_duties.legend()
        
    def _calculate_duties(self, angles, magnitude=1.0):
        """Calculate duty cycles for given angles."""
        duties_a = (magnitude * np.cos(np.deg2rad(angles)) + 1) / 2
        duties_b = (magnitude * np.cos(np.deg2rad(angles) - 2*np.pi/3) + 1) / 2
        duties_c = (magnitude * np.cos(np.deg2rad(angles) + 2*np.pi/3) + 1) / 2
        return duties_a, duties_b, duties_c
        
    def _update_plot(self, frame, magnitude):
        """Update function for animation."""
        angle_deg = frame % 360
        angle_rad = np.deg2rad(angle_deg)
        
        x = magnitude * np.cos(angle_rad)
        y = magnitude * np.sin(angle_rad)
        
        if self.vector_line:
            self.vector_line.remove()
        self.vector_line = self.ax_vector.quiver(0, 0, x, y, angles='xy', 
                                               scale_units='xy', scale=1, 
                                               color='red', width=0.005)
        
        duties = self._calculate_duties(np.array([angle_deg]), magnitude)
        
        if self.current_angle_line:
            self.current_angle_line.remove()
        self.current_angle_line = self.ax_duties.axvline(x=angle_deg, 
                                                        color='k', 
                                                        linestyle='--', 
                                                        alpha=0.5)
            
        if self.duty_points:
            for point in self.duty_points:
                point.remove()
        
        self.duty_points = [
            self.ax_duties.plot(angle_deg, duties[0], 'ro', markersize=8)[0],
            self.ax_duties.plot(angle_deg, duties[1], 'go', markersize=8)[0],
            self.ax_duties.plot(angle_deg, duties[2], 'bo', markersize=8)[0]
        ]
        
        self.ax_vector.set_title(f'Space Vector Diagram\nAngle: {angle_deg:.1f}°')
        self.ax_duties.set_title(f'Phase Duty Cycles\nA: {duties[0][0]:.3f}, '
                                f'B: {duties[1][0]:.3f}, C: {duties[2][0]:.3f}')
        
    def run_animation(self, magnitude=0.8, speed_factor=1.0, pause_at_angles=None):
        """
        Run the SVM visualization animation.
        
        Args:
            magnitude: Vector magnitude (0 to 1)
            speed_factor: Animation speed multiplier (>1 for faster, <1 for slower)
            pause_at_angles: List of angles where animation should pause (in degrees)
        """
        frames = np.arange(0, 3600)
        interval = self.update_interval / speed_factor
        
        def animate(frame):
            self._update_plot(frame, magnitude)
            
            if pause_at_angles:
                current_angle = frame % 360
                if current_angle in pause_at_angles:
                    plt.pause(1.0)
                
        ani = FuncAnimation(self.fig, animate, frames=frames,
                          interval=interval, repeat=True)
        
        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    viz = SpaceVectorVisualization(update_interval_ms=50)
    
    viz.run_animation(
        magnitude=0.99,          
        speed_factor=0.5,
        pause_at_angles=[0, 60, 120, 180, 240, 300]
    )
