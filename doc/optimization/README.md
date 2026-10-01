# Optimization

General-purpose optimization algorithms for parameter fitting, training, and control.

## Algorithms

| Algorithm                                        | Description                                                                                 |
|--------------------------------------------------|---------------------------------------------------------------------------------------------|
| [Bayesian Optimization](BayesianOptimization.md) | Gradient-free global optimization using Gaussian Process surrogate and Expected Improvement |
| [Gradient Descent](Optimizer.md)                 | Iterative batch optimization via gradient-based weight updates                              |
| [SGD / Adam Step Optimisers](StepOptimizer.md)   | Stateful per-step SGD (with optional momentum/Nesterov) and Adam with bias correction       |
