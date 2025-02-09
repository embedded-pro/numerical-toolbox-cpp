#include "estimators/LinearRegression.hpp"
#include <iostream>
#include <random>
#include <sciplot/sciplot.hpp>
#include <vector>

using namespace sciplot;

namespace
{
    template<typename T, std::size_t Samples>
    class DataGenerator
    {
    public:
        static void GenerateData(math::Matrix<T, Samples, 2>& X,
            math::Matrix<T, Samples, 1>& y,
            const std::array<T, 2>& coefficients = { 2.5f, 1.8f },
            T intercept = 1.0f,
            T noise = 0.5f)
        {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::normal_distribution<T> noise_dist(0.0f, noise);
            std::uniform_real_distribution<T> x_dist(-5.0f, 5.0f);

            for (size_t i = 0; i < Samples; ++i)
            {
                X.at(i, 0) = x_dist(gen);
                X.at(i, 1) = x_dist(gen);
                y.at(i, 0) = intercept +
                             coefficients[0] * X.at(i, 0) +
                             coefficients[1] * X.at(i, 1) +
                             noise_dist(gen);
            }
        }
    };
}

int main()
{
    constexpr std::size_t N = 500;
    constexpr std::size_t F = 2;

    math::Matrix<float, N, F> X;
    math::Matrix<float, N, 1> y;
    const std::array<float, 2> true_coefficients = { 2.5f, 1.8f };
    const float true_intercept = 1.0f;
    DataGenerator<float, N>::GenerateData(X, y, true_coefficients, true_intercept);

    estimators::LinearRegression<float, N, F> model;
    model.Fit(X, y);

    const auto& coefficients = model.Coefficients();
    float fitted_intercept = coefficients.at(0, 0);
    float fitted_coef1 = coefficients.at(1, 0);
    float fitted_coef2 = coefficients.at(2, 0);

    std::cout << "True Parameters:\n";
    std::cout << "Intercept: " << true_intercept << "\n";
    std::cout << "Coefficient 1: " << true_coefficients[0] << "\n";
    std::cout << "Coefficient 2: " << true_coefficients[1] << "\n\n";

    std::cout << "Fitted Parameters:\n";
    std::cout << "Intercept: " << fitted_intercept << "\n";
    std::cout << "Coefficient 1: " << fitted_coef1 << "\n";
    std::cout << "Coefficient 2: " << fitted_coef2 << "\n\n";

    std::vector<float> x1_points(N);
    std::vector<float> x2_points(N);
    std::vector<float> y_points(N);
    std::vector<float> y_pred(N);

    float y_mean = 0.0f;
    for (size_t i = 0; i < N; ++i)
    {
        x1_points[i] = X.at(i, 0);
        x2_points[i] = X.at(i, 1);
        y_points[i] = y.at(i, 0);
        y_mean += y_points[i];

        math::Matrix<float, 2, 1> x_input;
        x_input.at(0, 0) = x1_points[i];
        x_input.at(1, 0) = x2_points[i];
        y_pred[i] = model.Predict(x_input);
    }
    y_mean /= N;

    float ss_tot = 0.0f;
    float ss_res = 0.0f;
    std::vector<float> residuals(N);

    for (size_t i = 0; i < N; ++i)
    {
        residuals[i] = y_points[i] - y_pred[i];
        ss_tot += (y_points[i] - y_mean) * (y_points[i] - y_mean);
        ss_res += residuals[i] * residuals[i];
    }

    float r_squared = 1.0f - (ss_res / ss_tot);
    std::cout << "R-squared: " << r_squared << "\n";

    Plot actualVsPredicted;
    actualVsPredicted.xlabel("Predicted Values");
    actualVsPredicted.ylabel("Actual Values");
    actualVsPredicted.grid().show();
    actualVsPredicted.legend().show();

    float min_y = *std::min_element(y_points.begin(), y_points.end());
    float max_y = *std::max_element(y_points.begin(), y_points.end());
    std::vector<float> diagonal = { min_y, max_y };
    actualVsPredicted.drawCurve(diagonal, diagonal)
        .label("Perfect Prediction")
        .lineWidth(2);

    actualVsPredicted.drawPoints(y_pred, y_points)
        .label("Data Points")
        .pointType(7);

    Figure fig1 = { { actualVsPredicted } };
    fig1.size(800, 600);
    fig1.save("MLR - ActualVsPredicted.pdf");

    Plot residualPlot1;
    residualPlot1.xlabel("Feature 1 (X1)");
    residualPlot1.ylabel("Residuals");
    residualPlot1.grid().show();
    residualPlot1.drawPoints(x1_points, residuals)
        .label("Residuals vs X1")
        .pointType(7);

    Plot residualPlot2;
    residualPlot2.xlabel("Feature 2 (X2)");
    residualPlot2.ylabel("Residuals");
    residualPlot2.grid().show();
    residualPlot2.drawPoints(x2_points, residuals)
        .label("Residuals vs X2")
        .pointType(7);

    Figure fig2 = { { residualPlot1, residualPlot2 } };
    fig2.size(1200, 400);
    fig2.save("MLR - Residuals.pdf");

    Plot featurePlot1;
    featurePlot1.xlabel("Feature 1 (X1)");
    featurePlot1.ylabel("Target (Y)");
    featurePlot1.grid().show();
    featurePlot1.drawPoints(x1_points, y_points)
        .label("X1 vs Y")
        .pointType(7);

    Plot featurePlot2;
    featurePlot2.xlabel("Feature 2 (X2)");
    featurePlot2.ylabel("Target (Y)");
    featurePlot2.grid().show();
    featurePlot2.drawPoints(x2_points, y_points)
        .label("X2 vs Y")
        .pointType(7);

    Figure fig3 = { { featurePlot1, featurePlot2 } };
    fig3.size(1200, 400);
    fig3.save("MLR - Features.pdf");

    return 0;
}
