from simulator.controllers.SpaceVectorModulation import SpaceVectorModulation, RotatingFrame
from simulator.examples.SpaceVectorVisualization import SpaceVectorVisualization
import numpy as np
import time

def run_svm_examples():
    """Run different examples showcasing Space Vector Modulation visualization."""
    
    print("Space Vector Modulation Interactive Visualization Examples")
    print("-" * 50)
    
    # Create SVM and visualization instances
    svm = SpaceVectorModulation()  # Using your SVM implementation
    viz = SpaceVectorVisualization(update_interval_ms=50)
    
    # Example 1: Normal speed rotation
    print("\nExample 1: Normal speed rotation with 80% magnitude")
    print("Using actual SVM calculations for duty cycles")
    print("Press Ctrl+C to move to next example")
    try:
        viz.run_animation(
            magnitude=0.8,
            speed_factor=1.0,
            svm_instance=svm  # Pass SVM instance to visualization
        )
    except KeyboardInterrupt:
        pass
        
    # Example 2: Slow rotation with sector analysis
    print("\nExample 2: Slow rotation with pauses at sector boundaries")
    print("Watch how the SVM-calculated duty cycles change at sector transitions")
    print("Press Ctrl+C to move to next example")
    try:
        viz.run_animation(
            magnitude=0.8,
            speed_factor=0.3,  # Slower for analysis
            pause_at_angles=[0, 60, 120, 180, 240, 300],
            svm_instance=svm
        )
    except KeyboardInterrupt:
        pass
        
    # Example 3: High-speed rotation
    print("\nExample 3: High-speed rotation")
    print("Notice the smooth transitions between sectors")
    print("Press Ctrl+C to move to next example")
    try:
        viz.run_animation(
            magnitude=0.8,
            speed_factor=2.0,  # Faster rotation
            svm_instance=svm
        )
    except KeyboardInterrupt:
        pass
        
    # Example 4: Different magnitudes using actual SVM
    magnitudes = [0.2, 0.4, 0.6, 0.8, 1.0]
    for magnitude in magnitudes:
        print(f"\nExample 4: Showing magnitude = {magnitude}")
        print("Notice how magnitude affects SVM duty cycle calculations")
        print("Press Ctrl+C to see next magnitude")
        try:
            viz.run_animation(
                magnitude=magnitude,
                speed_factor=1.0,
                svm_instance=svm
            )
        except KeyboardInterrupt:
            continue

    # Example 5: Verification of SVM calculations
    print("\nExample 5: Verification of specific angles")
    test_angles = [0, 30, 60, 90, 120, 150]
    magnitude = 0.8
    
    print("\nTesting specific angles with SVM implementation:")
    for angle_deg in test_angles:
        # Create d-q voltage input
        angle_rad = np.deg2rad(angle_deg)
        vd = magnitude * np.cos(angle_rad)
        vq = magnitude * np.sin(angle_rad)
        dq_voltage = RotatingFrame(d=vd, q=vq)
        
        # Get SVM output
        output = svm.generate(dq_voltage, angle_deg / 360.0)
        
        print(f"\nAngle: {angle_deg}°")
        print(f"d-q input: ({vd:.3f}, {vq:.3f})")
        print(f"Duty cycles - A: {output.a:.3f}, B: {output.b:.3f}, C: {output.c:.3f}")

    print("\nExamples completed!")

if __name__ == "__main__":
    run_svm_examples()
