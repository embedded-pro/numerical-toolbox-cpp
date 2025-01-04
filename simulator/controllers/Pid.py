from ..math.QNumber import QNumber
from dataclasses import dataclass
from typing import Generic, TypeVar, Optional
import sys
from collections import deque

# Type variable for number types (float or QNumber)
T = TypeVar('T', float, QNumber)

@dataclass
class PidTunings(Generic[T]):
    """Tuning parameters for PID controller."""
    kp: T  # Proportional gain
    ki: T  # Integral gain
    kd: T  # Derivative gain

@dataclass
class PidLimits(Generic[T]):
    """Output limits for PID controller."""
    min: T  # Minimum output value
    max: T  # Maximum output value

class Pid(Generic[T]):
    def __init__(self, 
                 tunnings: PidTunings[T],
                 limits: PidLimits[T],
                 auto_mode: bool = True):
        """
        Initialize PID controller using z-transform difference equation.
        
        y[n] = y[n-1] + A0*x[n] + A1*x[n-1] + A2*x[n-2]
        where:
        A0 = kp + ki + kd
        A1 = -kp - 2*kd
        A2 = kd
        """
        # Validate limits
        if not limits.min < limits.max:
            sys.exit("Invalid limits: max must be greater than min")
            
        # Store configuration
        self._tunnings = tunnings
        self._limits = limits
        self._auto_mode = auto_mode
        
        # Calculate coefficients
        self._A0 = tunnings.kp + tunnings.ki + tunnings.kd
        self._A1 = -tunnings.kp - (2 * tunnings.kd)
        self._A2 = tunnings.kd
        
        # Initialize state variables
        self._prev_output = self._create_zero()
        self._error_history = deque([self._create_zero()] * 3, maxlen=3)  # x[n], x[n-1], x[n-2]
        self._set_point: Optional[T] = None
        
    def _create_zero(self) -> T:
        """Create a zero value of the appropriate numeric type."""
        return 0.0
        
    def set_point(self, set_point: T):
        """Set the desired target value."""
        self._set_point = set_point
        
    def enable(self):
        """Enable automatic mode."""
        if not self._auto_mode:
            self.reset()
        self._auto_mode = True
        
    def disable(self):
        """Disable automatic mode."""
        self._auto_mode = False
        
    def set_limits(self, limits: PidLimits[T]):
        """Set new output limits."""
        if not limits.min < limits.max:
            sys.exit("Invalid limits: max must be greater than min")
        self._limits = limits
        
    def set_tunnings(self, tunnings: PidTunings[T]):
        """Set new tuning parameters and recalculate coefficients."""
        self._tunnings = tunnings
        # Recalculate coefficients
        self._A0 = tunnings.kp + tunnings.ki + tunnings.kd
        self._A1 = -tunnings.kp - (2 * tunnings.kd)
        self._A2 = tunnings.kd
        
    def clamp(self, input_value: T) -> T:
        """Clamp a value between output limits."""
        if input_value > self._limits.max:
            return self._limits.max
        if input_value < self._limits.min:
            return self._limits.min
        return input_value
        
    def process(self, measured_process_variable: T) -> T:
        """
        Process one iteration of the PID control algorithm using z-transform equation.
        
        Args:
            measured_process_variable: Current process measurement
            
        Returns:
            T: New control output
        """
        if not self._set_point or not self._auto_mode:
            return self._prev_output
            
        # Calculate error
        error = self._set_point - measured_process_variable
        
        # Update error history (x[n] values)
        self._error_history.appendleft(error)
        
        # Implement z-transform difference equation
        # y[n] = y[n-1] + A0*x[n] + A1*x[n-1] + A2*x[n-2]
        output = (self._prev_output + 
                 self._A0 * self._error_history[0] +  # x[n]
                 self._A1 * self._error_history[1] +  # x[n-1]
                 self._A2 * self._error_history[2])   # x[n-2]
        
        # Apply anti-windup through output clamping
        output = self.clamp(output)
        
        # Update state
        self._prev_output = output
        
        return output
        
    def reset(self):
        """Reset controller state."""
        self._prev_output = self._create_zero()
        self._error_history = deque([self._create_zero()] * 3, maxlen=3)

    @property
    def coefficients(self) -> dict:
        """Get the current z-transform coefficients."""
        return {
            'A0': self._A0,
            'A1': self._A1,
            'A2': self._A2
        }
