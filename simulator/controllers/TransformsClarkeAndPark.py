from ..math.QNumber import QNumber
from dataclasses import dataclass
from typing import Generic, TypeVar, Union
import math

# Type variable for number types (float, QNumber)
T = TypeVar('T', float, QNumber)

@dataclass
class ThreePhase(Generic[T]):
    """Three-phase quantities (a, b, c)."""
    a: T
    b: T
    c: T

@dataclass
class TwoPhase(Generic[T]):
    """Two-phase quantities (alpha, beta)."""
    alpha: T
    beta: T

@dataclass
class RotatingFrame(Generic[T]):
    """Rotating frame quantities (d, q)."""
    d: T
    q: T

def create_const(value: float, example_type: T) -> Union[float, QNumber]:
    """Create a constant of the appropriate numeric type."""
    if isinstance(example_type, QNumber):
        return QNumber(value, example_type.fractional_bits)  # Scale constants for QNumber
    return value

class TrigonometricFunctions(Generic[T]):
    """
    Provides trigonometric functions for the supported number types.
    All angles are normalized between 0 and 1 (0 = 0 rad, 1 = 2π rad).
    """
    
    @staticmethod
    def sine(theta: T) -> T:
        """Compute sine for normalized angle [0,1]."""
        angle_rad = 2.0 * math.pi * (theta.to_float() if isinstance(theta, QNumber) else theta)
        if isinstance(theta, QNumber):
            return QNumber(math.sin(angle_rad) * 0.99, theta.fractional_bits)
        return math.sin(angle_rad)
    
    @staticmethod
    def cosine(theta: T) -> T:
        """Compute cosine for normalized angle [0,1]."""
        angle_rad = 2.0 * math.pi * (theta.to_float() if isinstance(theta, QNumber) else theta)
        if isinstance(theta, QNumber):
            return QNumber(math.cos(angle_rad) * 0.99, theta.fractional_bits)
        return math.cos(angle_rad)

class Clarke(Generic[T]):
    """Clarke transform implementation."""
    
    def forward(self, input_phases: ThreePhase[T]) -> TwoPhase[T]:
        """Forward Clarke transform."""
        # Create constants based on input type
        two_thirds = create_const(2/3, input_phases.a)
        one_half = create_const(0.5, input_phases.a)
        inv_sqrt3 = create_const(1/math.sqrt(3), input_phases.a)
        
        bc_sum = input_phases.b + input_phases.c
        
        return TwoPhase(
            alpha=two_thirds * (input_phases.a - one_half * bc_sum),
            beta=inv_sqrt3 * (input_phases.b - input_phases.c)
        )
    
    def inverse(self, input_phases: TwoPhase[T]) -> ThreePhase[T]:
        """Inverse Clarke transform."""
        one_half = create_const(0.5, input_phases.alpha)
        sqrt3_div2 = create_const(math.sqrt(3)/2, input_phases.alpha)
        
        alpha_half = one_half * input_phases.alpha
        beta_sqrt3_half = sqrt3_div2 * input_phases.beta
        
        return ThreePhase(
            a=input_phases.alpha,
            b=-alpha_half + beta_sqrt3_half,
            c=-alpha_half - beta_sqrt3_half
        )

class Park(Generic[T]):
    """Park transform implementation."""
    
    def __init__(self):
        """Initialize Park transform with trigonometric functions."""
        self.trig = TrigonometricFunctions()
    
    def forward(self, input_phases: TwoPhase[T], theta: T) -> RotatingFrame[T]:
        """
        Forward Park transform.
        theta must be normalized between 0 and 1 (0 = 0 rad, 1 = 2π rad)
        """
        cos_theta = self.trig.cosine(theta)
        sin_theta = self.trig.sine(theta)
        
        alpha_cos = input_phases.alpha * cos_theta
        beta_sin = input_phases.beta * sin_theta
        alpha_sin = input_phases.alpha * sin_theta
        beta_cos = input_phases.beta * cos_theta
        
        return RotatingFrame(
            d=alpha_cos + beta_sin,
            q=-alpha_sin + beta_cos
        )
    
    def inverse(self, input_frame: RotatingFrame[T], theta: T) -> TwoPhase[T]:
        """
        Inverse Park transform.
        theta must be normalized between 0 and 1 (0 = 0 rad, 1 = 2π rad)
        """
        cos_theta = self.trig.cosine(theta)
        sin_theta = self.trig.sine(theta)
        
        d_cos = input_frame.d * cos_theta
        q_sin = input_frame.q * sin_theta
        d_sin = input_frame.d * sin_theta
        q_cos = input_frame.q * cos_theta
        
        return TwoPhase(
            alpha=d_cos - q_sin,
            beta=d_sin + q_cos
        )

class ClarkePark(Generic[T]):
    """Combined Clarke-Park transform implementation."""
    
    def __init__(self):
        """Initialize with Clarke and Park transforms."""
        self.clarke = Clarke()
        self.park = Park()
    
    def forward(self, input_phases: ThreePhase[T], theta: T) -> RotatingFrame[T]:
        """
        Forward Clarke-Park transform.
        theta must be normalized between 0 and 1 (0 = 0 rad, 1 = 2π rad)
        """
        return self.park.forward(self.clarke.forward(input_phases), theta)
    
    def inverse(self, input_frame: RotatingFrame[T], theta: T) -> ThreePhase[T]:
        """
        Inverse Clarke-Park transform.
        theta must be normalized between 0 and 1 (0 = 0 rad, 1 = 2π rad)
        """
        return self.clarke.inverse(self.park.inverse(input_frame, theta))
