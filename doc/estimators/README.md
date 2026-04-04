# Estimators

Statistical estimation algorithms for fitting models to observed data and making predictions.

## Offline Estimators (Batch)

| Algorithm                                                         | Description                                                             |
|-------------------------------------------------------------------|-------------------------------------------------------------------------|
| [Linear Regression](LinearRegression.md)                         | Ordinary least-squares regression using the normal equation             |
| [Yule-Walker](YuleWalker.md)                                     | Autoregressive model parameter estimation via the Yule-Walker equations |
| [Expectation-Maximization](ExpectationMaximization.md)           | EM algorithm for Kalman filter parameter identification (Shumway-Stoffer) |

## Online Estimators (Streaming)

| Algorithm                                           | Description                                                              |
|-----------------------------------------------------|--------------------------------------------------------------------------|
| [Recursive Least Squares](RecursiveLeastSquares.md) | Sample-by-sample parameter estimation with exponential forgetting factor |
