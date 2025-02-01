#include "filters/active/KalmanFilter.hpp"
#include <cmath>
#include <iostream>
#include <random>
#include <sciplot/sciplot.hpp>
#include <vector>

using namespace sciplot;

namespace
{
    class SignalGenerator
    {
    public:
        static std::vector<std::pair<float, float>> GenerateTrajectory(
            std::size_t numPoints,
            float dt,
            float processNoise,
            float measurementNoise)
        {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::normal_distribution<float> process_noise(0.0f, processNoise);
            std::normal_distribution<float> measurement_noise(0.0f, measurementNoise);

            std::vector<std::pair<float, float>> measurements;

            float true_position = 0.0f;
            float true_velocity = 2.0f;

            for (std::size_t i = 0; i < numPoints; ++i)
            {
                true_velocity += process_noise(gen);
                true_position += true_velocity * dt;

                measurements.push_back({ true_position, true_position + measurement_noise(gen) });
            }

            return measurements;
        }
    };
}

int main()
{
    constexpr std::size_t numPoints = 100;
    const float dt = 0.1f;
    const float processNoise = 0.1f;
    const float measurementNoise = 0.5f;

    auto trajectory = SignalGenerator::GenerateTrajectory(numPoints, dt, processNoise, measurementNoise);

    constexpr std::size_t StateSize = 2;
    constexpr std::size_t MeasurementSize = 1;

    filters::KalmanFilter<float, StateSize, MeasurementSize>::StateVector initialState;
    initialState[0] = trajectory[0].second;
    initialState[1] = 0.0f;

    filters::KalmanFilter<float, StateSize, MeasurementSize>::StateMatrix initialCovariance;
    initialCovariance = filters::KalmanFilter<float, StateSize, MeasurementSize>::StateMatrix::Identity();
    initialCovariance *= 1.0f;

    filters::KalmanFilter<float, StateSize, MeasurementSize> kalmanFilter(initialState, initialCovariance);

    filters::KalmanFilter<float, StateSize, MeasurementSize>::StateMatrix F;
    F.at(0, 0) = 1.0f;
    F.at(0, 1) = dt;
    F.at(1, 0) = 0.0f;
    F.at(1, 1) = 1.0f;
    kalmanFilter.SetStateTransition(F);

    filters::KalmanFilter<float, StateSize, MeasurementSize>::MeasurementMatrix H;
    H.at(0, 0) = 1.0f;
    H.at(0, 1) = 0.0f;
    kalmanFilter.SetMeasurementMatrix(H);

    filters::KalmanFilter<float, StateSize, MeasurementSize>::StateMatrix Q;
    Q = filters::KalmanFilter<float, StateSize, MeasurementSize>::StateMatrix::Identity();
    Q *= processNoise * processNoise;
    kalmanFilter.SetProcessNoise(Q);

    filters::KalmanFilter<float, StateSize, MeasurementSize>::MeasurementCovariance R;
    R.at(0, 0) = measurementNoise * measurementNoise;
    kalmanFilter.SetMeasurementNoise(R);

    std::vector<float> timePoints;
    std::vector<float> truePositions;
    std::vector<float> measuredPositions;
    std::vector<float> estimatedPositions;
    std::vector<float> estimatedVelocities;
    std::vector<float> positionUncertainties;

    for (std::size_t i = 0; i < numPoints; ++i)
    {
        kalmanFilter.Predict();

        filters::KalmanFilter<float, StateSize, MeasurementSize>::MeasurementVector measurement;
        measurement[0] = trajectory[i].second;
        kalmanFilter.Update(measurement);

        timePoints.push_back(i * dt);
        truePositions.push_back(trajectory[i].first);
        measuredPositions.push_back(trajectory[i].second);
        estimatedPositions.push_back(kalmanFilter.GetState()[0]);
        estimatedVelocities.push_back(kalmanFilter.GetState()[1]);
        positionUncertainties.push_back(std::sqrt(kalmanFilter.GetCovariance().at(0, 0)));
    }

    float maxError = 0.0f;
    float totalError = 0.0f;

    for (std::size_t i = 0; i < numPoints; ++i)
    {
        float error = std::abs(truePositions[i] - estimatedPositions[i]);
        maxError = std::max(maxError, error);
        totalError += error;
    }
    float avgError = totalError / numPoints;

    std::cout << "Kalman Filter Performance:\n";
    std::cout << "Maximum Position Error: " << maxError << "\n";
    std::cout << "Average Position Error: " << avgError << "\n";

    Plot positionPlot;
    positionPlot.xlabel("Time (s)");
    positionPlot.ylabel("Position");
    positionPlot.grid().show();
    positionPlot.legend().show();
    positionPlot.drawCurve(timePoints, truePositions).label("True Position");
    positionPlot.drawCurve(timePoints, measuredPositions).label("Measured Position");
    positionPlot.drawCurve(timePoints, estimatedPositions).label("Kalman Estimate");

    Plot velocityPlot;
    velocityPlot.xlabel("Time (s)");
    velocityPlot.ylabel("Velocity");
    velocityPlot.grid().show();
    velocityPlot.legend().show();
    velocityPlot.drawCurve(timePoints, estimatedVelocities).label("Estimated Velocity");

    Plot uncertaintyPlot;
    uncertaintyPlot.xlabel("Time (s)");
    uncertaintyPlot.ylabel("Position Uncertainty (1σ)");
    uncertaintyPlot.grid().show();
    uncertaintyPlot.legend().show();
    uncertaintyPlot.drawCurve(timePoints, positionUncertainties).label("Position Uncertainty");

    Figure figure = { { positionPlot, velocityPlot, uncertaintyPlot } };
    figure.title("Kalman Filter - Position and Velocity Estimation");
    figure.size(800, 900);
    figure.save("Kalman Filter - Analysis.pdf");

    return 0;
}
