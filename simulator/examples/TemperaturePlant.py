class TemperaturePlant:
    def __init__(self, sample_time: float = 0.1):
        """
        Initialize temperature plant model.
        Transfer function: G(z) = (0.1z^-1)/(1 - 0.9z^-1)
        
        Args:
            sample_time: Sampling time in seconds
        """
        self.sample_time = sample_time
        self.prev_output = 20.0  # Starting at room temperature
        self.prev_input = 0.0
        
        # Plant parameters (z-transform coefficients)
        self.b1 = 0.1  # Numerator coefficient
        self.a1 = 0.9  # Denominator coefficient
        
    def update(self, control_input: float) -> float:
        """
        Update plant state with new control input.
        
        Args:
            control_input: Control signal from PID
            
        Returns:
            float: New temperature
        """
        # Implement z-transform difference equation
        output = self.a1 * self.prev_output + self.b1 * self.prev_input
        
        # Update state variables
        self.prev_output = output
        self.prev_input = control_input
        
        return output
