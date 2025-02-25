# Regularization Implementation Guide

## Process Overview

The following diagram illustrates how regularization affects neural network training:

![Regularization Methods](RegularizationMethods.svg)

The diagram shows how regularization:
1. Penalizes model complexity to prevent overfitting
2. Modifies the loss landscape to find more generalizable solutions
3. Creates different effects on model parameters (L1 vs L2)
4. Influences the optimization process during training

## Mathematical Background

### Regularization Purpose

Regularization adds a penalty term to the loss function to discourage overly complex models. For a model with parameters θ and loss function L, the regularized loss L_reg is:

$$L_{reg}(θ) = L(θ) + \lambda R(θ)$$

where:
- L(θ) is the original loss function
- R(θ) is the regularization term
- λ (lambda) is the regularization strength

### Types of Regularization

Different regularization methods create different effects on model parameters:

**L1 Regularization (Lasso):**
$$R_{L1}(θ) = \sum_{i=1}^{n} |θ_i|$$

**L2 Regularization (Ridge):**
$$R_{L2}(θ) = \frac{1}{2}\sum_{i=1}^{n} θ_i^2$$

## Implementation Details

### Class Structure

```cpp
template<typename QNumberType, std::size_t Size>
class Regularization
{
public:
    using Vector = math::Vector<QNumberType, Size>;
    virtual QNumberType Calculate(const Vector& parameters) const = 0;
};
```

### Key Implementations

1. **L1 Regularization (Lasso)**:
   - Encourages sparse models by driving some parameters to zero
   - Feature selection effect by eliminating irrelevant features
   - Sum of absolute parameter values
   **Mathematical Definition:**
   $$R_{L1}(θ) = \lambda \sum_{i=1}^{n} |θ_i|$$
   **Gradient with respect to parameters:**
   $$\frac{\partial R_{L1}}{\partial θ_i} = \lambda \cdot \text{sign}(θ_i)$$
   ```cpp
   template<typename QNumberType, std::size_t Size>
   class L1 : public Regularization<QNumberType, Size>
   {
   public:
       using Vector = typename Regularization<QNumberType, Size>::Vector;

       explicit L1(QNumberType lambda);
       
       QNumberType Calculate(const Vector& parameters) const override
       {
           QNumberType sum = QNumberType(0.0f);

           for (std::size_t i = 0; i < Size; ++i)
               sum += parameters[i] < QNumberType(0.0f) ? 
                      -parameters[i] : parameters[i];

           return lambda * sum;
       }

   private:
       QNumberType lambda;
   };
   ```

2. **L2 Regularization (Ridge)**:
   - Penalizes large parameter values
   - Promotes more uniform weight distributions
   - Sum of squared parameter values
   **Mathematical Definition:**
   $$R_{L2}(θ) = \frac{\lambda}{2} \sum_{i=1}^{n} θ_i^2$$
   **Gradient with respect to parameters:**
   $$\frac{\partial R_{L2}}{\partial θ_i} = \lambda \cdot θ_i$$
   ```cpp
   template<typename QNumberType, std::size_t Size>
   class L2 : public Regularization<QNumberType, Size>
   {
   public:
       using Vector = typename Regularization<QNumberType, Size>::Vector;

       explicit L2(QNumberType lambda);
       
       QNumberType Calculate(const Vector& parameters) const override
       {
           QNumberType sum = QNumberType(0.0f);

           for (std::size_t i = 0; i < Size; ++i)
               sum += parameters[i] * parameters[i];

           return QNumberType(math::ToFloat(lambda * sum) / 2.0f);
       }

   private:
       QNumberType lambda;
   };
   ```

## Usage Guide

### Basic Usage

```cpp
// Define parameter size
constexpr std::size_t parameterSize = 100;
using FloatType = float;

// Create regularization objects with different strengths
L1<FloatType, parameterSize> l1Regularization(0.01f);  // Sparse model
L2<FloatType, parameterSize> l2Regularization(0.01f);  // Smooth model

// Create parameter vector
math::Vector<FloatType, parameterSize> parameters;
// Fill parameters with model weights...

// Calculate regularization penalty
FloatType l1Penalty = l1Regularization.Calculate(parameters);
FloatType l2Penalty = l2Regularization.Calculate(parameters);
```

### Integration with Loss Functions

```cpp
// Create model and loss components
Model<FloatType, inputSize, outputSize, ...> model;
auto parameters = model.GetParameters();

// Create loss function
MeanSquaredError<FloatType, model.TotalParameters> lossFunction(target);

// Create regularization
L2<FloatType, model.TotalParameters> regularization(0.01f);

// Compute total loss (original loss + regularization)
FloatType totalLoss = lossFunction.Cost(parameters) + 
                      regularization.Calculate(parameters);

// When computing gradients for optimization
auto lossGradient = lossFunction.Gradient(parameters);
auto regularizationGradient = regularization.Gradient(parameters);

// Combined gradient for parameter updates
Vector<FloatType, model.TotalParameters> totalGradient;
for (std::size_t i = 0; i < model.TotalParameters; ++i)
    totalGradient[i] = lossGradient[i] + regularizationGradient[i];
```

### Choosing Lambda Values

```cpp
// Weak regularization (prioritize fitting training data)
L2<FloatType, parameterSize> weakRegularization(0.0001f);

// Moderate regularization (balance fit and complexity)
L2<FloatType, parameterSize> moderateRegularization(0.01f);

// Strong regularization (prioritize model simplicity)
L2<FloatType, parameterSize> strongRegularization(0.1f);
```

## Best Practices

1. **Regularization Selection**:
   - L1 regularization when sparse models are desired (feature selection)
   - L2 regularization for general model stability and preventing large weights
   - Combined L1+L2 (Elastic Net) for a mix of both effects

2. **Lambda Selection**:
   - Use cross-validation to find optimal lambda values
   - Start with small values (0.0001-0.001) and gradually increase
   - Different model sizes may require different lambda scales

3. **Implementation Considerations**:
   - Scale lambda based on dataset size
   - Apply regularization only to weights, not biases
   - Consider learning rate interaction with regularization strength

## L1 vs L2 Regularization Comparison

| Aspect            | L1 Regularization                                   | L2 Regularization                     |
|-------------------|-----------------------------------------------------|---------------------------------------|
| Effect on Weights | Many weights become exactly zero                    | All weights become smaller, non-zero  |
| Sparsity          | Creates sparse models                               | Creates dense, small-weight models    |
| Feature Selection | Effectively removes irrelevant features             | Keeps all features but reduces impact |
| Loss Surface      | Non-differentiable at zero                          | Smooth, differentiable everywhere     |
| Best Use Case     | High-dimensional data with many irrelevant features | General purpose, prevents overfitting |
| Computational     | More complex optimization problem                   | Simpler, more stable optimization     |

## Performance Considerations

1. **Computation Efficiency**:
   - L2 regularization is computationally simpler than L1
   - Both can be vectorized for performance
   - Regularization computation is generally inexpensive compared to loss functions

2. **Numeric Stability**:
   - L1 requires special handling at zero (non-differentiable)
   - Both can be implemented with stable numeric operations

3. **Memory Usage**:
   - Both L1 and L2 regularization have minimal memory requirements
   - Only need to store the lambda parameter

## Other Regularization Techniques

Though not implemented in the provided code, other common regularization techniques include:

1. **Elastic Net**:
   - Combines L1 and L2 regularization
   - $$R_{elastic}(θ) = \lambda_1 \sum |θ_i| + \lambda_2 \sum θ_i^2$$
   - Balances sparsity and smoothness

2. **Dropout**:
   - Randomly deactivates neurons during training
   - Acts as an implicit ensemble method
   - Prevents co-adaptation of neurons

3. **Early Stopping**:
   - Stop training when validation performance degrades
   - Implicit regularization by limiting training iterations
   - Prevents memorization of training data

## Limitations and Future Improvements

1. Current limitations:
   - Limited to L1 and L2 regularization
   - Fixed parameter size at compile time
   - No gradient method in base class
   - Fixed lambda during training

2. Possible extensions:
   - Elastic Net implementation (combined L1+L2)
   - Parameter-specific regularization strengths
   - Adaptive regularization during training
   - Structured regularization for specific parameter groups
   - Regularization schedules that vary lambda over time
   - Support for other regularization techniques (orthogonal, spectral)

## Error Handling

1. Static assertions verify:
   - Valid numeric types
   - Appropriate parameter sizes

2. Implementation considerations:
   - Ensure lambda values are positive
   - Handle numeric overflow for very large parameters or lambda values
   - Consider NaN and infinity checks for robustness
