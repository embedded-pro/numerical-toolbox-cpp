import numpy as np
import matplotlib.pyplot as plt
from simulator.controllers.TransformsClarkeAndPark import Clarke, Park, ClarkePark, ThreePhase, TwoPhase, RotatingFrame
from simulator.math.QNumber import QNumber

def plot_transforms(time, abc, alpha_beta, dq, title_prefix=""):
    """Helper function to plot transform results."""
    plt.figure(figsize=(15, 10))
    
    plt.subplot(311)
    plt.plot(time, [x.to_float() if isinstance(x, QNumber) else x for x in abc.a], label='a')
    plt.plot(time, [x.to_float() if isinstance(x, QNumber) else x for x in abc.b], label='b')
    plt.plot(time, [x.to_float() if isinstance(x, QNumber) else x for x in abc.c], label='c')
    plt.title(f'{title_prefix}Three-Phase Quantities')
    plt.xlabel('Time (s)')
    plt.ylabel('Magnitude')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(312)
    plt.plot(time, [x.to_float() if isinstance(x, QNumber) else x for x in alpha_beta.alpha], label='alpha')
    plt.plot(time, [x.to_float() if isinstance(x, QNumber) else x for x in alpha_beta.beta], label='beta')
    plt.title(f'{title_prefix}Clarke Transform (α-β)')
    plt.xlabel('Time (s)')
    plt.ylabel('Magnitude')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(313)
    plt.plot(time, [x.to_float() if isinstance(x, QNumber) else x for x in dq.d], label='d')
    plt.plot(time, [x.to_float() if isinstance(x, QNumber) else x for x in dq.q], label='q')
    plt.title(f'{title_prefix}Park Transform (d-q)')
    plt.xlabel('Time (s)')
    plt.ylabel('Magnitude')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()

def run_transform_example(t, f, q_bits, title_prefix):
    """Run transformation example for specified number of fractional bits."""
    print(f"Testing with QNumber ({q_bits} fractional bits)...")
    
    clarke_q = Clarke[QNumber]()
    park_q = Park[QNumber]()
    clarke_park_q = ClarkePark[QNumber]()
    
    abc_q_data = ThreePhase(
        a=[QNumber(np.cos(2*np.pi*(f*t_ % 1.0)) * 0.9, q_bits) for t_ in t],
        b=[QNumber(np.cos(2*np.pi*((f*t_ - 1/3) % 1.0)) * 0.9, q_bits) for t_ in t],
        c=[QNumber(np.cos(2*np.pi*((f*t_ + 1/3) % 1.0)) * 0.9, q_bits) for t_ in t]
    )
    
    alpha_beta_q_data = TwoPhase(alpha=[], beta=[])
    dq_q_data = RotatingFrame(d=[], q=[])
    
    for idx, time in enumerate(t):
        abc_q = ThreePhase(
            a=abc_q_data.a[idx],
            b=abc_q_data.b[idx],
            c=abc_q_data.c[idx]
        )

        if isinstance(abc_q.a, QNumber):
            print("abc_q is QNumber")

        if isinstance(abc_q.a, float):
            print("abc_q is float")
        
        alpha_beta_q = clarke_q.forward(abc_q)
        alpha_beta_q_data.alpha.append(alpha_beta_q.alpha)
        alpha_beta_q_data.beta.append(alpha_beta_q.beta)
        
        theta_q = QNumber(f*time % 1.0, q_bits)  
        dq_q = park_q.forward(alpha_beta_q, theta_q)
        dq_q_data.d.append(dq_q.d)
        dq_q_data.q.append(dq_q.q)
    
    plot_transforms(t, abc_q_data, alpha_beta_q_data, dq_q_data, title_prefix)
    
    return abc_q_data, alpha_beta_q_data, dq_q_data

def main():
    t = np.linspace(0, 0.1, 1000)  # 0.1 seconds, 1000 points
    f = 50  # Frequency in Hz
    
    print("Testing with float numbers...")
    
    clarke_float = Clarke[float]()
    park_float = Park[float]()
    clarke_park_float = ClarkePark[float]()
    
    abc_float_data = ThreePhase(
        a=[np.cos(2*np.pi*f*t_) for t_ in t],
        b=[np.cos(2*np.pi*f*t_ - 2*np.pi/3) for t_ in t],
        c=[np.cos(2*np.pi*f*t_ + 2*np.pi/3) for t_ in t]
    )
    
    alpha_beta_float_data = TwoPhase(alpha=[], beta=[])
    dq_float_data = RotatingFrame(d=[], q=[])
    
    for idx, time in enumerate(t):
        abc_float = ThreePhase(
            a=abc_float_data.a[idx],
            b=abc_float_data.b[idx],
            c=abc_float_data.c[idx]
        )
        
        alpha_beta_float = clarke_float.forward(abc_float)
        alpha_beta_float_data.alpha.append(alpha_beta_float.alpha)
        alpha_beta_float_data.beta.append(alpha_beta_float.beta)
        
        theta_float = 2*np.pi*f*time
        dq_float = park_float.forward(alpha_beta_float, theta_float)
        dq_float_data.d.append(dq_float.d)
        dq_float_data.q.append(dq_float.q)
    
    plot_transforms(t, abc_float_data, alpha_beta_float_data, dq_float_data, "Float: ")
    
    abc_q15, alpha_beta_q15, dq_q15 = run_transform_example(t, f, 15, "Q15: ")
    abc_q31, alpha_beta_q31, dq_q31 = run_transform_example(t, f, 31, "Q31: ")
    
    def calc_max_error(q_data, float_data):
        return max(abs(x.to_float() if isinstance(x, QNumber) else x - 
                      (y if isinstance(y, float) else y.to_float())) 
                  for x, y in zip(q_data, float_data))
    
    print("\nMaximum errors compared to float:")
    print("\nQ15 errors:")
    print(f"Phase A: {calc_max_error(abc_q15.a, abc_float_data.a):.2e}")
    print(f"Alpha: {calc_max_error(alpha_beta_q15.alpha, alpha_beta_float_data.alpha):.2e}")
    print(f"D: {calc_max_error(dq_q15.d, dq_float_data.d):.2e}")
    
    print("\nQ31 errors:")
    print(f"Phase A: {calc_max_error(abc_q31.a, abc_float_data.a):.2e}")
    print(f"Alpha: {calc_max_error(alpha_beta_q31.alpha, alpha_beta_float_data.alpha):.2e}")
    print(f"D: {calc_max_error(dq_q31.d, dq_float_data.d):.2e}")

if __name__ == "__main__":
    main()
