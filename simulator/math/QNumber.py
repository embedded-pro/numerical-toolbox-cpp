class QNumber:
    """
    A fixed-point number implementation supporting both Q15 and Q31 formats.
    
    The QNumber class implements fixed-point arithmetic with configurable number of fractional bits.
    It supports two common formats:
    - Q15: 16-bit representation with 15 fractional bits (1 sign bit, 15 fractional bits)
    - Q31: 32-bit representation with 31 fractional bits (1 sign bit, 31 fractional bits)
    
    The value range for both formats is [-1.0, 1.0).
    
    Example usage:
        # Create Q15 numbers
        a = QNumber(0.5, fractional_bits=15)
        b = QNumber(-0.25, fractional_bits=15)
        
        # Perform arithmetic operations
        c = a + b  # Results in Q15(0.25)
        d = a * b  # Results in Q15(-0.125)
    """
    
    def __init__(self, value: float, fractional_bits: int):
        """
        Initialize a QNumber with a float value and specified number of fractional bits.
        
        Args:
            value: Float value to convert to fixed-point. Must be in range [-1.0, 1.0)
            fractional_bits: Number of fractional bits (15 for Q15 or 31 for Q31)
        
        Raises:
            ValueError: If value is outside valid range or fractional_bits is not 15 or 31
        """
        if not isinstance(fractional_bits, int) or fractional_bits not in (15, 31):
            raise ValueError("fractional_bits must be either 15 or 31")
        
        self.fractional_bits = fractional_bits
        self._max_value = (1 << (fractional_bits + 1)) - 1  # Maximum positive value
        self._min_value = -(1 << (fractional_bits + 1))    # Minimum negative value
        
        # Validate input range
        if not -1.0 <= value < 1.0:
            raise ValueError(f"Value {value} out of range [-1.0, 1.0)")
        
        # Convert float to fixed-point
        self._value = self._float_to_fixed(value)

    def _float_to_fixed(self, value: float) -> int:
        """Convert float to fixed-point representation."""
        return round(value * (1 << self.fractional_bits))

    def to_float(self) -> float:
        """Convert fixed-point value back to float."""
        return self._value / (1 << self.fractional_bits)

    @classmethod
    def from_raw(cls, raw_value: int, fractional_bits: int) -> 'QNumber':
        """
        Create a QNumber from a raw fixed-point value.
        
        This is useful when you want to create a QNumber from an already
        converted fixed-point value without going through float conversion.
        
        Args:
            raw_value: The raw fixed-point value
            fractional_bits: Number of fractional bits (15 or 31)
            
        Returns:
            A new QNumber instance
        """
        instance = cls(0.0, fractional_bits)
        instance._value = raw_value
        return instance

    @property
    def raw_value(self) -> int:
        """Get the raw fixed-point value."""
        return self._value

    def __add__(self, other: 'QNumber') -> 'QNumber':
        """Add two QNumbers of the same format."""
        self._validate_operation(other)
        result = self._value + other._value
        # Check for overflow
        if result > self._max_value or result < self._min_value:
            raise OverflowError("Addition result out of range")
        return QNumber.from_raw(result, self.fractional_bits)

    def __sub__(self, other: 'QNumber') -> 'QNumber':
        """Subtract two QNumbers of the same format."""
        self._validate_operation(other)
        result = self._value - other._value
        # Check for overflow
        if result > self._max_value or result < self._min_value:
            raise OverflowError("Subtraction result out of range")
        return QNumber.from_raw(result, self.fractional_bits)

    def __mul__(self, other: 'QNumber') -> 'QNumber':
        """Multiply two QNumbers of the same format."""
        self._validate_operation(other)
        # Perform multiplication in higher precision
        temp = (self._value * other._value) >> self.fractional_bits
        # Check for overflow
        if temp > self._max_value or temp < self._min_value:
            raise OverflowError("Multiplication result out of range")
        return QNumber.from_raw(temp, self.fractional_bits)

    def __truediv__(self, other: 'QNumber') -> 'QNumber':
        """Divide two QNumbers of the same format."""
        self._validate_operation(other)
        if other._value == 0:
            raise ZeroDivisionError("Division by zero")
        # Perform division in higher precision
        temp = (self._value << self.fractional_bits) // other._value
        # Check for overflow
        if temp > self._max_value or temp < self._min_value:
            raise OverflowError("Division result out of range")
        return QNumber.from_raw(temp, self.fractional_bits)

    def __neg__(self) -> 'QNumber':
        """Negate a QNumber."""
        if self._value == self._min_value:
            raise OverflowError("Cannot negate minimum value")
        return QNumber.from_raw(-self._value, self.fractional_bits)

    def __eq__(self, other: 'QNumber') -> bool:
        """Compare two QNumbers for equality."""
        self._validate_operation(other)
        return self._value == other._value

    def __lt__(self, other: 'QNumber') -> bool:
        """Compare if this QNumber is less than another."""
        self._validate_operation(other)
        return self._value < other._value

    def __le__(self, other: 'QNumber') -> bool:
        """Compare if this QNumber is less than or equal to another."""
        self._validate_operation(other)
        return self._value <= other._value

    def __gt__(self, other: 'QNumber') -> bool:
        """Compare if this QNumber is greater than another."""
        self._validate_operation(other)
        return self._value > other._value

    def __ge__(self, other: 'QNumber') -> bool:
        """Compare if this QNumber is greater than or equal to another."""
        self._validate_operation(other)
        return self._value >= other._value

    def _validate_operation(self, other: 'QNumber'):
        """Validate that two QNumbers have the same format before operations."""
        if not isinstance(other, QNumber):
            raise TypeError(f"Unsupported operand type: {type(other)}")
        if self.fractional_bits != other.fractional_bits:
            raise ValueError("Cannot operate on QNumbers with different formats")

    def __str__(self) -> str:
        """Return string representation of the QNumber."""
        return f"Q{self.fractional_bits}({self.to_float()})"

    def __repr__(self) -> str:
        """Return detailed string representation of the QNumber."""
        return f"QNumber(value={self.to_float()}, fractional_bits={self.fractional_bits})"
