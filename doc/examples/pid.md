# PID Temperature Control System Example

This document explains the implementation of a PID (Proportional-Integral-Derivative) controller for temperature control, comparing different numeric representations (float, Q31, and Q15).

## Thermal System Model

The thermal system is modeled using a simplified first-order differential equation:

```
dT/dt = h*P - c*(T - Ta)

where:
T  = Current temperature
Ta = Ambient temperature (22°C)
P  = Heating power input (0 to 1)
h  = Heating coefficient (0.05)
c  = Cooling coefficient (0.02)
```

This model captures two main phenomena:
1. **Heating**: Rate proportional to input power (h*P)
2. **Cooling**: Rate proportional to temperature difference with ambient (c*(T - Ta))

## PID Controller

The PID controller generates a control signal based on three terms:

```
u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt

where:
u(t) = Control output
e(t) = Error (setpoint - measured temperature)
Kp   = Proportional gain
Ki   = Integral gain
Kd   = Derivative gain
```

Each term serves a specific purpose:
- **Proportional (Kp)**: Immediate response proportional to current error
- **Integral (Ki)**: Eliminates steady-state error by accumulating past errors
- **Derivative (Kd)**: Anticipates future errors by considering rate of change

### Controller Parameters

The controller uses these tuned parameters:
```cpp
Kp = 0.5   // Moderate proportional response
Ki = 0.02  // Small integral action to prevent oscillation
Kd = 0.1   // Moderate derivative action for damping
```

## Number Representations

The system implements the PID controller using three different numeric types:

### Float (32-bit floating-point)
- Range: ±3.4×10³⁸
- Precision: ~7 decimal digits
- Advantages: Wide range, easy to use
- Disadvantages: More computational resources, potential rounding errors

### Q31 (32-bit fixed-point)
- Range: -1 to +0.99999...
- Resolution: 2⁻³¹ ≈ 4.66×10⁻¹⁰
- Format: 1 sign bit, 31 fractional bits
- Advantages: High precision, deterministic arithmetic
- Disadvantages: Limited range

### Q15 (16-bit fixed-point)
- Range: -1 to +0.99999...
- Resolution: 2⁻¹⁵ ≈ 3.05×10⁻⁵
- Format: 1 sign bit, 15 fractional bits
- Advantages: Memory efficient, fast computation
- Disadvantages: Lower precision than Q31

## Implementation Details

### Temperature Scaling
- System operates between 25°C (initial) and 60°C (target)
- Control output scaled to 0-1 range for heating power

### PID Update Process
1. Calculate error: `e = setpoint - current_temperature`
2. Calculate proportional term: `P = Kp * e`
3. Update integral term: `I += Ki * e * dt`
4. Calculate derivative term: `D = Kd * (e - last_error) / dt`
5. Sum terms: `output = P + I + D`
6. Clamp output to valid range: `output = clamp(output, -1, 1)`
7. Scale to heating power: `power = (output + 1) * 0.5`

### Fixed-Point Considerations
- Q31 and Q15 operations require careful scaling
- Multiplication results need shifting
- Overflow checking is important
- Temperature values must be normalized to [-1, 1] range

## Performance Comparison

The three implementations show slightly different characteristics:

1. **Float**
   - Smoothest control response
   - Most computationally intensive
   - Best for prototyping

2. **Q31**
   - Very close to float performance
   - More efficient computation
   - Good for embedded systems with 32-bit processors

3. **Q15**
   - Slightly less precise
   - Most memory efficient
   - Suitable for 16-bit microcontrollers

## Example Usage

```cpp
// Configure PID controller
TemperatureController<float>::Config config{
    0.5f,    // Kp
    0.02f,   // Ki
    0.1f,    // Kd
    60.0f,   // Target temperature
    std::chrono::microseconds(100000)  // Sample time
};

// Create controller and thermal system
TemperatureController controller(config);
ThermalSystem system(25.0f);  // Start at 25°C

// Control loop
float temperature = system.update(0);
float control = controller.process(temperature);
```

## Code Organization

```
controllers/
├── Pid.hpp           // Generic PID controller template
└── examples/
    └── Pid.cpp       // Temperature control example

math/
├── Q15.hpp          // Q15 fixed-point implementation
├── Q31.hpp          // Q31 fixed-point implementation
└── QNumber.hpp      // Generic fixed-point template
```

This implementation demonstrates the practical application of PID control while comparing different numeric representations, making it useful for both educational purposes and real-world applications.
