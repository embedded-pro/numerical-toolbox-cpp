#include "numerical/estimators/YuleWalker.hpp"
#include "numerical/solvers/LevinsonDurbin.hpp"
#include <iostream>
#include <random>
#include <sciplot/sciplot.hpp>
#include <vector>

using namespace sciplot;

namespace
{
    template<typename T, std::size_t Samples, std::size_t Order>
    class ARDataGenerator
    {
    public:
        static void GenerateData(math::Matrix<T, Samples, Order>& X,
            math::Matrix<T, Samples, 1>& y,
            const std::array<T, Order>& true_coefficients,
            T noise_std = 0.1f)
        {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::normal_distribution<T> noise(0.0f, noise_std);

            std::vector<T> time_series(Samples + Order);
            for (size_t i = 0; i < Order; ++i)
                time_series[i] = noise(gen);

            for (size_t i = Order; i < Samples + Order; ++i)
            {
                time_series[i] = noise(gen);
                for (size_t j = 0; j < Order; ++j)
                    time_series[i] += true_coefficients[j] * time_series[i - j - 1];
            }

            for (size_t i = 0; i < Samples; ++i)
            {
                y.at(i, 0) = time_series[i + Order];
                for (size_t j = 0; j < Order; ++j)
                    X.at(i, j) = time_series[i + Order - j - 1];
            }
        }
    };
}

int main()
{
    constexpr std::size_t N = 500;
    constexpr std::size_t P = 3;

    math::Matrix<float, N, P> X;
    math::Matrix<float, N, 1> y;
    const std::array<float, P> true_coefficients = { 0.6f, -0.2f, 0.1f };

    ARDataGenerator<float, N, P>::GenerateData(X, y, true_coefficients);

    solvers::LevinsonDurbin<float, P> solver;
    estimators::YuleWalker<float, N, P> yw_estimator(solver);

    yw_estimator.Fit(X, y);

    const auto& coefficients = yw_estimator.Coefficients();

    std::cout << "True AR Coefficients:\n";
    for (size_t i = 0; i < P; ++i)
        std::cout << "phi_" << i + 1 << ": " << true_coefficients[i] << "\n";

    std::cout << "\nEstimated AR Coefficients:\n";
    for (size_t i = 0; i < P; ++i)
        std::cout << "phi_" << i + 1 << ": " << coefficients.at(i, 0) << "\n";

    std::vector<float> predictions(N);
    std::vector<float> actual_values(N);
    std::vector<float> time_points(N);
    std::vector<float> residuals(N);

    for (size_t i = 0; i < N; ++i)
    {
        math::Matrix<float, P, 1> x_input;
        for (size_t j = 0; j < P; ++j)
            x_input.at(j, 0) = X.at(i, j);

        predictions[i] = yw_estimator.Predict(x_input);
        actual_values[i] = y.at(i, 0);
        time_points[i] = static_cast<float>(i);
        residuals[i] = actual_values[i] - predictions[i];
    }

    float mse = 0.0f;
    for (size_t i = 0; i < N; ++i)
        mse += residuals[i] * residuals[i];
    mse /= N;

    std::cout << "\nMean Squared Error: " << mse << "\n";

    Plot timeSeriesPlot;
    timeSeriesPlot.xlabel("Time");
    timeSeriesPlot.ylabel("Value");
    timeSeriesPlot.grid().show();
    timeSeriesPlot.legend().show();

    timeSeriesPlot.drawCurve(time_points, actual_values)
        .label("Actual")
        .lineWidth(2);

    timeSeriesPlot.drawCurve(time_points, predictions)
        .label("Predicted")
        .lineWidth(2);

    Figure fig1 = { { timeSeriesPlot } };
    fig1.size(800, 600);
    fig1.save("YuleWalker - TimeSeries.pdf");

    Plot residualPlot;
    residualPlot.xlabel("Time");
    residualPlot.ylabel("Residual");
    residualPlot.grid().show();

    residualPlot.drawPoints(time_points, residuals)
        .label("Residuals")
        .pointType(7);

    Figure fig2 = { { residualPlot } };
    fig2.size(800, 400);
    fig2.save("YuleWalker - Residuals.pdf");

    Plot acfPlot;
    acfPlot.xlabel("Lag");
    acfPlot.ylabel("Autocorrelation");
    acfPlot.grid().show();

    const int max_lag = 20;
    std::vector<float> lags(max_lag);
    std::vector<float> acf(max_lag);

    float mean_residual = 0.0f;
    for (float residual : residuals)
        mean_residual += residual;
    mean_residual /= static_cast<float>(residuals.size());

    float variance = 0.0f;
    for (float residual : residuals)
        variance += (residual - mean_residual) * (residual - mean_residual);
    variance /= static_cast<float>(residuals.size());

    for (int lag = 0; lag < max_lag; ++lag)
    {
        lags[lag] = static_cast<float>(lag);
        float sum = 0.0f;
        for (size_t i = 0; i < residuals.size() - lag; ++i)
            sum += (residuals[i] - mean_residual) * (residuals[i + lag] - mean_residual);
        acf[lag] = sum / (static_cast<float>(residuals.size()) * variance);
    }

    acfPlot.drawPoints(lags, acf)
        .label("ACF")
        .pointType(7);

    float conf_bound = 1.96f / std::sqrt(static_cast<float>(N));
    std::vector<float> upper_bound(max_lag, conf_bound);
    std::vector<float> lower_bound(max_lag, -conf_bound);

    acfPlot.drawCurve(lags, upper_bound)
        .label("95% Confidence Bounds")
        .lineStyle(2);
    acfPlot.drawCurve(lags, lower_bound)
        .lineStyle(2);

    Figure fig3 = { { acfPlot } };
    fig3.size(800, 400);
    fig3.save("YuleWalker - ACF.pdf");

    return 0;
}
