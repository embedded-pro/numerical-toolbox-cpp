# Neural Networks Implementation Guide

## Process Overview

The following diagram illustrates the key components of a neural network:

![Neural Network Architecture](NeuralNetworkArchitecture.png)

The diagram shows the main elements:
1. Input layer receives data
2. Hidden layers perform transformations
3. Output layer produces predictions
4. Forward propagation processes data through the network
5. Loss function compares predictions to targets
6. Backward propagation updates weights

### Neural Network Fundamentals

A neural network is a computational model inspired by the human brain. For an input vector x, the network computes an output vector y through a series of transformations:

$\mathbf{y} = f_n(\mathbf{W}_n \cdot f_{n-1}(\mathbf{W}_{n-1} \cdot \ldots f_1(\mathbf{W}_1 \cdot \mathbf{x} + \mathbf{b}_1) \ldots + \mathbf{b}_{n-1}) + \mathbf{b}_n)$

This can be written more clearly as:

$\mathbf{y} = f_n(\mathbf{z}_n)$

where:

$\mathbf{z}_i = \mathbf{W}_i \cdot \mathbf{a}_{i-1} + \mathbf{b}_i$
$\mathbf{a}_i = f_i(\mathbf{z}_i)$
$\mathbf{a}_0 = \mathbf{x}$
```

where:
- W_i are weight matrices
- b_i are bias vectors
- f is an activation function

### Training Process

Neural networks learn through optimization. The process involves:
```
1. Forward Propagation: z_i = W_i * a_{i-1} + b_i
                         a_i = f(z_i)
2. Loss Calculation: L = loss(a_n, y_target)
3. Backward Propagation: dL/dW_i, dL/db_i
4. Weight Update: W_i = W_i - η *dL/dW_i
                  b_i = b_i - η* dL/db_i
```

where:
- a_i is the activation at layer i
- z_i is the weighted input at layer i
- η is the learning rate
- L is the loss function

## Implementation Details

### Class Structure

```cpp
template<
    typename NumberType,
    std::size_t InputSize,
    std::size_t OutputSize,
    typename Optimizer,
    typename LossFunction
>
class NeuralNetwork {
public:
    explicit NeuralNetwork(
        std::vector<Layer<NumberType>>& layers,
        Optimizer& optimizer,
        LossFunction& loss);
        
    Vector<NumberType>& Forward(const Vector<NumberType>& input);
    void Backward(const Vector<NumberType>& target);
    void Train(const DataSet<NumberType>& trainingData, int epochs);
};
```

### Key Components

1. [**Layers**](layer/Layer.md)
   - Base `Layer` class template with configurable input size, output size, and parameter size
   - Derived layer types like `Dense` implementing specific transformations
   - Compile-time size verification to ensure proper layer connections
   ```cpp
   template<typename QNumberType, std::size_t InputSize_, std::size_t OutputSize_, std::size_t ParameterSize_>
   class Layer {
   public:
       // Interface methods for forward/backward pass and parameter management
       virtual void Forward(const InputVector& input) = 0;
       virtual InputVector& Backward(const OutputVector& output_gradient) = 0;
       virtual ParameterVector& Parameters() const = 0;
       virtual void SetParameters(const ParameterVector& parameters) = 0;
   };
   ```

2. [**Activation Functions**](activation/Activation.md):
   - Defined as a separate class hierarchy with `Forward` and `Backward` methods
   - Applied within layer implementations
   - Enable non-linear transformations essential for deep learning
   ```cpp
   template<typename QNumberType>
   class ActivationFunction {
   public:
       virtual QNumberType Forward(QNumberType x) const = 0;
       virtual QNumberType Backward(QNumberType x) const = 0;
   };
   ```

3. [**Loss Functions**](losses/Loss.md):
   - Measure prediction error with `Cost` method
   - Provide gradients with respect to parameters via `Gradient` method
   - Guide optimization process based on learning objectives
   ```cpp
   template<typename QNumberType, std::size_t NumberOfFeatures>
   class Loss {
   public:
       virtual QNumberType Cost(const Vector& parameters) = 0;
       virtual Vector Gradient(const Vector& parameters) = 0;
   };
   ```

4. [**Optimizers**](optimizer/Optimizer.md):
   - Minimize loss functions by updating parameters
   - Return optimized parameters, final cost, and iteration count
   - Different algorithms with specialized convergence properties
   ```cpp
   template<typename QNumberType, std::size_t NumberOfFeatures>
   class Optimizer {
   public:
       struct Result {
           Vector parameters;
           QNumberType finalCost;
           std::size_t iterations;
       };
       
       virtual const Result& Minimize(const Vector& initialGuess, 
                                      Loss<QNumberType, NumberOfFeatures>& loss) = 0;
   };
   ```

5. [**Regularization**](regularization/Regularization.md):
   - Penalizes model complexity to prevent overfitting
   - Common types include L1 (sparsity-inducing) and L2 (weight decay)
   - Applied during training as an additional term in the loss function
   ```cpp
   template<typename QNumberType, std::size_t Size>
   class Regularization {
   public:
       virtual QNumberType Calculate(const Vector& parameters) const = 0;
   };
   ```

## Usage Guide

### Basic Usage

```cpp
// Define network parameters
constexpr std::size_t inputSize = 784;    // MNIST input
constexpr std::size_t hiddenSize = 128;   // Hidden layer size
constexpr std::size_t outputSize = 10;    // 10 digits
using FloatType = float;

// Create activation functions
ReLU<FloatType> relu;
Softmax<FloatType> softmax;

// Initialize random weights
std::random_device rd;
std::mt19937 gen(rd());
std::normal_distribution<float> dist(0.0f, 0.1f);

math::Matrix<FloatType, hiddenSize, inputSize> weights1;
math::Matrix<FloatType, outputSize, hiddenSize> weights2;
// Fill weights with random values using dist and gen

// Create layer factory functions
auto layer1 = make_layer<Dense<FloatType, inputSize, hiddenSize>>(weights1, relu);
auto layer2 = make_layer<Dense<FloatType, outputSize, hiddenSize>>(weights2, softmax);

// Create neural network model
Model<FloatType, inputSize, outputSize, 
    Dense<FloatType, inputSize, hiddenSize>,
    Dense<FloatType, hiddenSize, outputSize>
> model(layer1, layer2);

// Create loss function
MeanSquaredError<FloatType, model.TotalParameters> loss(/* loss parameters */);

// Create regularization (optional)
L2Regularization<FloatType, model.TotalParameters> regularizer(0.01f);

// Create optimizer
GradientDescent<FloatType, model.TotalParameters> optimizer;

// Get initial parameters
auto initialParameters = model.GetParameters();

// Train model
model.Train(optimizer, loss, initialParameters);

// Make predictions
auto testInput = math::Vector<FloatType, inputSize>{/* test data */};
auto prediction = model.Forward(testInput);
```

### Example: Image Classification

```cpp
// Create a CNN for image classification
constexpr std::size_t imageWidth = 32;
constexpr std::size_t imageHeight = 32;
constexpr std::size_t channels = 3;      // RGB
constexpr std::size_t numClasses = 10;   // 10 categories

// Define network architecture
ConvLayer<float, imageWidth, imageHeight, channels, 16> conv1(ReLU<float>());
MaxPoolLayer<float, 16> pool1(2);  // 2x2 pooling
ConvLayer<float, 16, 16, 16, 32> conv2(ReLU<float>());
MaxPoolLayer<float, 32> pool2(2);
FlattenLayer<float> flatten;
DenseLayer<float, 32*8*8, 128> fc1(ReLU<float>());
DenseLayer<float, 128, numClasses> fc2(Softmax<float>());

// Combine layers
std::vector<Layer<float>> layers = {
    conv1, pool1, conv2, pool2, flatten, fc1, fc2
};

// Create optimizer with momentum
Adam<float> optimizer(0.001f, 0.9f, 0.999f);
CrossEntropy<float> loss;

// Create and train network
NeuralNetwork<float, imageWidth*imageHeight*channels, numClasses> 
    network(layers, optimizer, loss);
    
network.Train(cifar10TrainingData, 50);
```

## Best Practices

1. **Architecture Design**:
   - Start simple and add complexity gradually
   - Use appropriate layer types for the data
   - Consider computational resources

2. **Hyperparameter Selection**:
   - Learning rate: typically 0.1 to 0.0001
   - Batch size: balance between 32 and 256
   - Layer sizes: problem-dependent

3. **Training Strategies**:
   - Monitor validation metrics
   - Implement early stopping
   - Use learning rate schedules
   - Apply regularization techniques

4. **Data Preprocessing**:
   - Normalize inputs
   - Shuffle training data
   - Use data augmentation when appropriate

## Common Applications

1. **Classification**:
   - Image recognition
   - Text categorization
   - Medical diagnosis

2. **Regression**:
   - Time series prediction
   - Value estimation
   - Function approximation

3. **Advanced Tasks**:
   - Object detection
   - Sequence generation
   - Reinforcement learning

## Performance Considerations

1. **Computation Efficiency**:
   - Use vectorized operations
   - Leverage hardware acceleration
   - Optimize memory access patterns

2. **Memory Management**:
   - Buffer reuse for activations
   - Gradient accumulation for large models
   - Mini-batch processing

3. **Numeric Stability**:
   - Use stable activation functions
   - Apply gradient clipping
   - Monitor for NaN values

## Forward Propagation in Detail

Forward propagation is the process of computing the network's output for a given input. In the `Dense` layer implementation, it works as follows:

1. **Store input** for later use in backward pass:
   ```cpp
   this->input = input;
   ```

2. **Compute weighted sum plus bias** for each neuron in the layer:
   ```cpp
   for (std::size_t i = 0; i < OutputSize; ++i) {
       preActivation[i] = biases[i];
       for (std::size_t j = 0; j < InputSize; ++j)
           preActivation[i] += weights.at(i, j) * input[j];
       
       // Apply activation function
       output[i] = activation.Forward(preActivation[i]);
   }
   ```

The output of each layer becomes the input to the next layer, creating a chain of computations from the input layer to the output layer. The `Model::Forward` method implements this chaining by sequentially calling each layer's `Forward` method.

## Backpropagation in Detail

Backpropagation computes gradients of the loss with respect to all parameters in the network using the chain rule. For the `Dense` layer:

1. **Compute gradient with respect to pre-activation values**:
   ```cpp
   for (std::size_t i = 0; i < OutputSize; ++i)
       preActivationGradient[i] = activation.Backward(output_gradient[i]);
   ```

2. **Compute gradient with respect to input** (for previous layer):
   ```cpp
   for (std::size_t j = 0; j < InputSize; ++j) {
       inputGradient[j] = QNumberType(0.0f);
       for (std::size_t i = 0; i < OutputSize; ++i)
           inputGradient[j] += weights.at(i, j) * preActivationGradient[i];
   }
   ```

3. **Compute gradients with respect to weights and biases**:
   ```cpp
   for (std::size_t i = 0; i < OutputSize; ++i) {
       for (std::size_t j = 0; j < InputSize; ++j)
           weightGradients.at(i, j) = preActivationGradient[i] * input[j];
           
       biasGradients[i] = preActivationGradient[i];
   }
   ```

The `Model::Backward` method implements the backpropagation through all layers in reverse order, ensuring that gradients flow properly from the output back to the input.

## Limitations and Future Improvements

1. Current limitations:
   - Fixed architecture after initialization
   - Template-based implementation requires compile-time knowledge of dimensions
   - Limited to supervised learning
   - Single-threaded implementation
   - CPU-only processing
   - Fixed-size matrices and vectors

2. Possible extensions:
   - Dynamic architecture changes
   - Runtime-configurable layer sizes
   - Unsupervised learning support
   - Distributed training
   - GPU acceleration
   - Transfer learning capabilities
   - Pruning and quantization
   - Automatic hyperparameter tuning
   - Additional layer types (Convolutional, Recurrent, etc.)
   - Advanced optimization techniques

## Error Handling

1. Static assertions verify:
   - Compatible layer dimensions
   - Valid numeric types
   - Appropriate activation functions

2. Runtime checks:
   - Input data validity
   - Gradient explosion prevention
   - Convergence monitoring
   - Numerical stability validation
