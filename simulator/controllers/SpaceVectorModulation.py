from dataclasses import dataclass
from typing import Generic, TypeVar
from ..math.QNumber import QNumber
import math
import sys
import logging

# Type variable for number types (float or QNumber)
T = TypeVar('T', float, QNumber)

@dataclass
class RotatingFrame(Generic[T]):
    """Rotating reference frame components."""
    d: T  # Direct axis component
    q: T  # Quadrature axis component

@dataclass
class SvmOutput(Generic[T]):
    """Space Vector Modulation output duty cycles."""
    a: T  # Phase A duty cycle
    b: T  # Phase B duty cycle
    c: T  # Phase C duty cycle

class SpaceVectorModulation(Generic[T]):
    def __init__(self, trig_functions=None):
        """
        Initialize Space Vector Modulation calculator.
        
        Args:
            trig_functions: Optional trigonometric functions provider for QNumber types
        """
        self._trig_functions = trig_functions
        # Initialize constants based on numeric type
        self._initialize_constants()
        
    def _initialize_constants(self):
        """Initialize constants based on whether we're using float or QNumber."""
        # Create a test number to determine the type
        test_num = self._create_number(0.0)
        
        if isinstance(test_num, QNumber):
            # For Q15/Q31 numbers, scale constants appropriately
            self._zero = self._create_number(0.0)
            self._one = self._create_number(0.2)  # Scaled as in C++ version
            self._half = self._create_number(0.1)
            self._inv_sqrt3 = self._create_number(0.115470053837925)
            self._sqrt3_div2 = self._create_number(0.1732050807568876)
            self._two_div_sqrt3 = self._create_number(0.2309401076758504)
            self._output_scale = self._create_number(0.2)
        else:
            # For float, use full precision
            self._zero = 0.0
            self._one = 1.0
            self._half = 0.5
            self._inv_sqrt3 = 1.0 / math.sqrt(3)
            self._sqrt3_div2 = math.sqrt(3) / 2.0
            self._two_div_sqrt3 = 2.0 / math.sqrt(3)
            self._output_scale = 1.0

    def _create_number(self, value: float) -> T:
        """Create a number of the appropriate type."""
        if hasattr(self, '_trig_functions') and self._trig_functions is not None:
            # If we have trig functions, we're using QNumber
            # Assuming the trig functions object has a way to create numbers
            return self._trig_functions.create_number(value)
        return float(value)

    def _sine(self, angle: T) -> T:
        """Calculate sine for the given normalized angle (0 to 1)."""
        if self._trig_functions is not None:
            return self._trig_functions.sine(angle)
        return math.sin(2.0 * math.pi * float(angle))

    def _cosine(self, angle: T) -> T:
        """Calculate cosine for the given normalized angle (0 to 1)."""
        if self._trig_functions is not None:
            return self._trig_functions.cosine(angle)
        return math.cos(2.0 * math.pi * float(angle))

    def _clamp_duty_cycle(self, duty: T) -> T:
        """Clamp duty cycle and apply output scaling."""
        if duty < self._zero:
            return self._zero
        if duty > self._one:
            return self._one / self._output_scale
        return duty / self._output_scale

    def _add_common_mode_injection(self, pattern: SvmOutput[T]) -> SvmOutput[T]:
        """Add common mode injection to improve DC bus utilization."""
        t0 = self._one - pattern.a - pattern.b - pattern.c
        t_com = t0 * self._half
        
        return SvmOutput(
            a=pattern.a + t_com,
            b=pattern.b + t_com,
            c=pattern.c + t_com
        )

    def _calculate_60_to_120_degrees(self, v_alpha: T, v_beta: T) -> SvmOutput[T]:
        """Calculate switching times for sector 2 (60° to 120°)."""
        t1 = v_alpha + v_beta * self._inv_sqrt3
        t2 = -v_alpha + v_beta * self._inv_sqrt3
        return self._add_common_mode_injection(SvmOutput(t2, t1 + t2, self._zero))

    def _calculate_120_to_180_degrees(self, v_alpha: T, v_beta: T) -> SvmOutput[T]:
        """Calculate switching times for sector 3 (120° to 180°)."""
        t1 = -v_alpha + v_beta * self._inv_sqrt3
        t2 = -v_alpha - v_beta * self._inv_sqrt3
        return self._add_common_mode_injection(SvmOutput(self._zero, t1 + t2, t2))

    def _calculate_180_to_240_degrees(self, v_alpha: T, v_beta: T) -> SvmOutput[T]:
        """Calculate switching times for sector 4 (180° to 240°)."""
        t1 = -v_alpha - v_beta * self._inv_sqrt3
        t2 = v_beta * self._two_div_sqrt3
        return self._add_common_mode_injection(SvmOutput(self._zero, t2, t1 + t2))

    def _calculate_240_to_300_degrees(self, v_alpha: T, v_beta: T) -> SvmOutput[T]:
        """Calculate switching times for sector 5 (240° to 300°)."""
        t1 = -v_alpha + v_beta * self._inv_sqrt3
        t2 = -v_beta * self._two_div_sqrt3
        return self._add_common_mode_injection(SvmOutput(t2, self._zero, t1 + t2))

    def _calculate_300_to_360_degrees(self, v_alpha: T, v_beta: T) -> SvmOutput[T]:
        """Calculate switching times for sector 6 (300° to 360°)."""
        t1 = v_alpha + v_beta * self._inv_sqrt3
        t2 = -v_alpha - v_beta * self._inv_sqrt3
        return self._add_common_mode_injection(SvmOutput(t1 + t2, self._zero, t2))

    def _calculate_0_to_60_degrees(self, v_alpha: T, v_beta: T) -> SvmOutput[T]:
        """Calculate switching times for sector 1 (0° to 60°)."""
        t1 = v_alpha - v_beta * self._inv_sqrt3
        t2 = v_beta * self._two_div_sqrt3
        return self._add_common_mode_injection(SvmOutput(t1 + t2, t2, self._zero))

    def _calculate_switching_times(self, v_alpha: T, v_beta: T) -> SvmOutput[T]:
        """Calculate switching times based on alpha-beta components."""
        v_ref_60 = (v_alpha * self._half - v_beta * self._sqrt3_div2)
        v_ref_120 = (-v_alpha * self._half - v_beta * self._sqrt3_div2)

        if v_ref_60 >= self._zero:
            return self._calculate_60_to_120_degrees(v_alpha, v_beta)
        if v_ref_120 >= self._zero:
            return self._calculate_120_to_180_degrees(v_alpha, v_beta)
        if -v_alpha >= self._zero:
            return self._calculate_180_to_240_degrees(v_alpha, v_beta)
        if -v_ref_60 >= self._zero:
            return self._calculate_240_to_300_degrees(v_alpha, v_beta)
        if -v_ref_120 >= self._zero:
            return self._calculate_300_to_360_degrees(v_alpha, v_beta)

        return self._calculate_0_to_60_degrees(v_alpha, v_beta)

    def generate(self, dq_voltage: RotatingFrame[T], scaled_theta: T) -> SvmOutput[T]:
        """
        Generate three-phase duty cycles using Space Vector Modulation.
        
        Args:
            dq_voltage: Voltage in rotating reference frame (d,q), values between -1 and 1
            scaled_theta: Normalized angle (0 to 1 representing 0 to 2π)
            
        Returns:
            SvmOutput: Three-phase duty cycles between 0 and 1
        """
        # Scale inputs
        d = dq_voltage.d * self._output_scale
        q = dq_voltage.q * self._output_scale

        # Calculate alpha-beta components
        cos_theta = self._cosine(scaled_theta)
        sin_theta = self._sine(scaled_theta)
        
        v_alpha = d * cos_theta - q * sin_theta
        v_beta = d * sin_theta + q * cos_theta

        # Calculate switching pattern
        pattern = self._calculate_switching_times(v_alpha, v_beta)
        
        # Clamp and scale outputs
        return SvmOutput(
            a=self._clamp_duty_cycle(pattern.a),
            b=self._clamp_duty_cycle(pattern.b),
            c=self._clamp_duty_cycle(pattern.c)
        )
