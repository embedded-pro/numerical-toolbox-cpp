# PID Controller Implementation Guide

## Mathematical Background

### The PID Control Law

A Proportional-Integral-Derivative (PID) controller is a control loop feedback mechanism widely used in industrial control systems. The controller continuously calculates an error value e(t) as the difference between a desired setpoint and a measured process variable, and applies a correction based on proportional, integral, and derivative terms.

The continuous-time PID control law is:

```
u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt
```

where:
- u(t) is the control signal
- e(t) is the error value
- Kp is the proportional gain
- Ki is the integral gain
- Kd is the derivative gain

### Digital Implementation

In digital systems, we need to discretize the continuous PID equation. Our implementation uses a discrete-time approximation where:

1. The integral term is approximated using numerical integration
2. The derivative term is approximated using finite differences
3. The control output is computed recursively

The discrete form implemented in the code is:

```
y[n] = y[n-1] + a0*x[n] + a1*x[n-1] + a2*x[n-2]
```

where:
- y[n] is the current output
- x[n] is the current error
- a0 = Kp + Ki + Kd
- a1 = -(Kp + 2Kd)
- a2 = Kd

## Implementation Details

### Class Structure

```cpp
template<typename QNumberType>
class Pid {
public:
    struct Tunnings {
        QNumberType kp;
        QNumberType ki;
        QNumberType kd;
    };

    struct Limits {
        QNumberType min;
        QNumberType max;
    };

    Pid(Tunnings tunnings, Limits limits, bool autoMode = true);
    
    // Core functionality
    void SetPoint(QNumberType setPoint);
    QNumberType Process(QNumberType measuredProcessVariable);
    
    // Configuration methods
    void Enable();
    void Disable();
    void SetLimits(Limits limits);
    void SetTunnings(Tunnings tunnings);
    void Reset();
};
```

### Key Components

1. **Tunnings Structure**: Holds the PID gains (Kp, Ki, Kd)
2. **Limits Structure**: Defines output saturation limits
3. **RecursiveBuffer**: Stores historical values for error and output
4. **Process Method**: Implements the core PID algorithm
5. **Auto/Manual Mode**: Supports switching between automatic and manual control

## Usage Guide

### Basic Usage

```cpp
// Create a PID instance
Pid<float>::Tunnings tunnings{
    .kp = 1.0f,
    .ki = 0.1f,
    .kd = 0.05f
};

Pid<float>::Limits limits{
    .min = -100.0f,
    .max = 100.0f
};

Pid<float> pid(tunnings, limits);

// Set the desired setpoint
pid.SetPoint(50.0f);

// In control loop
while (true) {
    float measuredValue = getMeasurement();
    float controlOutput = pid.Process(measuredValue);
    applyControl(controlOutput);
}
```

### Tuning the Controller

The PID gains affect the system response:
1. **Proportional (Kp)**: Provides quick response to error
2. **Integral (Ki)**: Eliminates steady-state error
3. **Derivative (Kd)**: Provides damping and reduces overshoot

Tuning guidelines:
1. Start with Ki = Kd = 0, and adjust Kp until acceptable response
2. Gradually increase Ki to eliminate steady-state error
3. Add Kd if needed to reduce overshoot
4. Always test with actual process dynamics

## Best Practices

1. **Output Limiting**: Always use appropriate min/max limits to prevent integral windup
2. **Numerical Precision**: Use appropriate QNumberType based on application needs
3. **Mode Handling**: Use Enable()/Disable() to switch between auto/manual modes
4. **Reset Handling**: Call Reset() when making significant changes to setpoint

## Common Applications

1. **Temperature Control**: Maintaining precise temperature in industrial processes
2. **Motor Control**: Speed and position control in robotics
3. **Process Control**: Flow rate, pressure, and level control
4. **Motion Control**: Precise positioning systems

## Example: Temperature Control

```cpp
// Configure PID for temperature control
Pid<float>::Tunnings tunnings{
    .kp = 2.0f,    // Aggressive proportional for quick response
    .ki = 0.1f,    // Small integral to eliminate steady state error
    .kd = 0.5f     // Moderate derivative to prevent overshoot
};

Pid<float>::Limits limits{
    .min = 0.0f,   // Cannot cool below ambient
    .max = 100.0f  // Maximum heater power
};

Pid<float> temperatureController(tunnings, limits);

// Set desired temperature
temperatureController.SetPoint(75.0f);

// Control loop
while (true) {
    float currentTemp = readTemperature();
    float heaterPower = temperatureController.Process(currentTemp);
    setHeaterPower(heaterPower);
    wait(100ms);
}
```

## Performance Considerations

1. **Recursive Buffers**: Efficient storage of historical values
2. **Template Implementation**: Compile-time optimizations
3. **Output Clamping**: Prevents integral windup
4. **Auto/Manual Mode**: Zero overhead when disabled

## Limitations and Future Improvements

1. Current implementation could be extended to support:
   - Anti-windup mechanisms
   - Derivative filter
   - Auto-tuning capabilities
   - Gain scheduling
   - Feed-forward control
2. Additional features that could be added:
   - Parameter validation
   - Runtime performance metrics
   - Adaptive control
   - Cascade control support

## Safety Considerations

1. **Output Limiting**: Always use appropriate limits to protect the process
2. **Mode Transitions**: Implement bumpless transfer between auto/manual modes
3. **Numerical Stability**: Monitor for overflow in integral term
4. **Fail-Safe Behavior**: Implement appropriate behavior for sensor failures
