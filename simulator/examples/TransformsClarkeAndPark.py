import numpy as np
import matplotlib.pyplot as plt
from typing import Generic, TypeVar, Union, Optional
from simulator.controllers.TransformsClarkeAndPark import Clarke, Park, ClarkePark, ThreePhase, TwoPhase, RotatingFrame
from simulator.math.QNumber import QNumber

def create_number(value: float, number_type: type, q_bits: int = None) -> Union[float, QNumber]:
    """Helper function to create either float or QNumber with proper scaling."""
    if number_type == QNumber:
        return QNumber(value * 0.9, q_bits)  # Scale by 0.9 for QNumber safety margin
    return value

def to_float(value: Union[float, QNumber]) -> float:
    """Convert any numeric type to float for plotting."""
    return value.to_float() if isinstance(value, QNumber) else value

def plot_transforms(time, abc, alpha_beta, dq, title_prefix=""):
    """Helper function to plot transform results."""
    plt.figure(figsize=(15, 10))
    
    # Plot three-phase quantities
    plt.subplot(311)
    plt.plot(time, [to_float(x) for x in abc.a], label='a')
    plt.plot(time, [to_float(x) for x in abc.b], label='b')
    plt.plot(time, [to_float(x) for x in abc.c], label='c')
    plt.title(f'{title_prefix}Three-Phase Quantities')
    plt.xlabel('Time (s)')
    plt.ylabel('Magnitude')
    plt.legend()
    plt.grid(True)
    
    # Plot alpha-beta quantities
    plt.subplot(312)
    plt.plot(time, [to_float(x) for x in alpha_beta.alpha], label='alpha')
    plt.plot(time, [to_float(x) for x in alpha_beta.beta], label='beta')
    plt.title(f'{title_prefix}Clarke Transform (α-β)')
    plt.xlabel('Time (s)')
    plt.ylabel('Magnitude')
    plt.legend()
    plt.grid(True)
    
    # Plot d-q quantities
    plt.subplot(313)
    plt.plot(time, [to_float(x) for x in dq.d], label='d')
    plt.plot(time, [to_float(x) for x in dq.q], label='q')
    plt.title(f'{title_prefix}Park Transform (d-q)')
    plt.xlabel('Time (s)')
    plt.ylabel('Magnitude')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()

def run_transform_example(t, f, number_type: type, q_bits: int = None, title_prefix=""):
    """Run transformation example for any numeric type."""
    type_name = f"QNumber ({q_bits} bits)" if number_type == QNumber else "float"
    print(f"Testing with {type_name}...")
    
    # Create transform instances
    clarke = Clarke[number_type]()
    park = Park[number_type]()
    clarke_park = ClarkePark[number_type]()
    
    # Generate three-phase quantities using normalized angles [0,1]
    abc_data = ThreePhase(
        a=[create_number(np.cos(2*np.pi*(f*t_ % 1.0)), number_type, q_bits) for t_ in t],
        b=[create_number(np.cos(2*np.pi*((f*t_ - 1/3) % 1.0)), number_type, q_bits) for t_ in t],
        c=[create_number(np.cos(2*np.pi*((f*t_ + 1/3) % 1.0)), number_type, q_bits) for t_ in t]
    )
    
    # Initialize storage for transformed values
    alpha_beta_data = TwoPhase(alpha=[], beta=[])
    dq_data = RotatingFrame(d=[], q=[])
    
    # Process each time step
    for idx, time in enumerate(t):
        # Create three-phase input
        abc = ThreePhase(
            a=abc_data.a[idx],
            b=abc_data.b[idx],
            c=abc_data.c[idx]
        )
        
        # Clarke transform
        alpha_beta = clarke.forward(abc)
        alpha_beta_data.alpha.append(alpha_beta.alpha)
        alpha_beta_data.beta.append(alpha_beta.beta)
        
        # Park transform (using normalized angle)
        theta = create_number(f*time % 1.0, number_type, q_bits)
        dq = park.forward(alpha_beta, theta)
        dq_data.d.append(dq.d)
        dq_data.q.append(dq.q)
    
    # Plot results
    plot_transforms(t, abc_data, alpha_beta_data, dq_data, title_prefix)
    
    return abc_data, alpha_beta_data, dq_data

def calc_max_error(data1, data2):
    """Calculate maximum error between two datasets."""
    return max(abs(to_float(x) - to_float(y)) for x, y in zip(data1, data2))

def main():
    # Setup simulation parameters
    t = np.linspace(0, 0.1, 1000)  # 0.1 seconds, 1000 points
    f = 50  # Frequency in Hz
    
    # Run examples for each type
    abc_float, alpha_beta_float, dq_float = run_transform_example(t, f, float, title_prefix="Float: ")
    abc_q15, alpha_beta_q15, dq_q15 = run_transform_example(t, f, QNumber, 15, "Q15: ")
    abc_q31, alpha_beta_q31, dq_q31 = run_transform_example(t, f, QNumber, 31, "Q31: ")
    
    # Print error analysis
    print("\nMaximum errors compared to float:")
    print("\nQ15 errors:")
    print(f"Phase A: {calc_max_error(abc_q15.a, abc_float.a):.2e}")
    print(f"Alpha: {calc_max_error(alpha_beta_q15.alpha, alpha_beta_float.alpha):.2e}")
    print(f"D: {calc_max_error(dq_q15.d, dq_float.d):.2e}")
    
    print("\nQ31 errors:")
    print(f"Phase A: {calc_max_error(abc_q31.a, abc_float.a):.2e}")
    print(f"Alpha: {calc_max_error(alpha_beta_q31.alpha, alpha_beta_float.alpha):.2e}")
    print(f"D: {calc_max_error(dq_q31.d, dq_float.d):.2e}")

if __name__ == "__main__":
    main()
