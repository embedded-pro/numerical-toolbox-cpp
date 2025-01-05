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

class TrigonometricFunctions(Generic[T]):
    """
    Provides trigonometric functions for the supported number types.
    For QNumber, input angle must be normalized between 0 and 1,
    where 0 represents 0 radians and 1 represents 2π radians.
    """
    
    @staticmethod
    def sine(theta: T) -> T:
        if isinstance(theta, QNumber):
            # Convert normalized angle [0,1] to radians [0,2π]
            angle_rad = theta.to_float() * 2.0 * math.pi
            return QNumber(math.sin(angle_rad) * 0.99, theta.fractional_bits)
        return math.sin(theta)
    
    @staticmethod
    def cosine(theta: T) -> T:
        if isinstance(theta, QNumber):
            # Convert normalized angle [0,1] to radians [0,2π]
            angle_rad = theta.to_float() * 2.0 * math.pi
            return QNumber(math.cos(angle_rad) * 0.99, theta.fractional_bits)
        return math.cos(theta)

class Clarke(Generic[T]):
    """Clarke transform implementation."""
    
    def forward(self, input_phases: ThreePhase[T]) -> TwoPhase[T]:
        """Forward Clarke transform."""
        bc_sum = input_phases.b + input_phases.c
        
        if isinstance(input_phases.a, QNumber):
            return TwoPhase(
                alpha=QNumber(2/3, input_phases.a.fractional_bits) * (input_phases.a - QNumber(0.5, input_phases.a.fractional_bits) * bc_sum),
                beta=QNumber(1/math.sqrt(3), input_phases.a.fractional_bits) * (input_phases.b - input_phases.c)
            )
        else:
            return TwoPhase(
                alpha=(2/3) * (input_phases.a - (0.5) * bc_sum),
                beta=(1/math.sqrt(3)) * (input_phases.b - input_phases.c)
            )
    
    def inverse(self, input_phases: TwoPhase[T]) -> ThreePhase[T]:
        """Inverse Clarke transform."""      
        if isinstance(input_phases.alpha, QNumber):  
            alpha_half = QNumber(0.5, input_phases.alpha.fractional_bits) * input_phases.alpha
            beta_sqrt3_half = QNumber(math.sqrt(3)/2, input_phases.alpha.fractional_bits) * input_phases.beta
        else:
            alpha_half = (0.5) * input_phases.alpha
            beta_sqrt3_half = (math.sqrt(3)/2) * input_phases.beta

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
        For QNumber, theta must be normalized between 0 and 1 (0 = 0 rad, 1 = 2π rad)
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
        For QNumber, theta must be normalized between 0 and 1 (0 = 0 rad, 1 = 2π rad)
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
        For QNumber, theta must be normalized between 0 and 1 (0 = 0 rad, 1 = 2π rad)
        """
        return self.park.forward(self.clarke.forward(input_phases), theta)
    
    def inverse(self, input_frame: RotatingFrame[T], theta: T) -> ThreePhase[T]:
        """
        Inverse Clarke-Park transform.
        For QNumber, theta must be normalized between 0 and 1 (0 = 0 rad, 1 = 2π rad)
        """
        return self.clarke.inverse(self.park.inverse(input_frame, theta))
