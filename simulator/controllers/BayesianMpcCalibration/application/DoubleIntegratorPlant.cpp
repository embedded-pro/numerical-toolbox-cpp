#include "simulator/controllers/BayesianMpcCalibration/application/DoubleIntegratorPlant.hpp"
#include <random>

namespace simulator::controllers
{
    DoubleIntegratorPlant::DoubleIntegratorPlant(const DoubleIntegratorConfig& config)
        : config_(config)
    {
        const float dt = config.dt;
        A_ = math::SquareMatrix<float, 2>{ { 1.0f, dt }, { 0.0f, 1.0f } };
        B_ = math::Matrix<float, 2, 1>{ { 0.5f * dt * dt }, { dt } };
    }

    math::SquareMatrix<float, 2> DoubleIntegratorPlant::GetA() const
    {
        return A_;
    }

    math::Matrix<float, 2, 1> DoubleIntegratorPlant::GetB() const
    {
        return B_;
    }

    void DoubleIntegratorPlant::GenerateObservations(
        std::size_t numSteps,
        uint32_t seed,
        std::array<MeasurementVector, MaxSteps>& observations,
        std::vector<float>& truePositions) const
    {
        std::mt19937 rng(seed);
        std::normal_distribution<float> processNoise(0.0f, config_.sigmaQ);
        std::normal_distribution<float> measurementNoise(0.0f, config_.sigmaR);

        StateVector state{ { 0.0f }, { 0.0f } };
        truePositions.clear();
        truePositions.reserve(numSteps);

        for (std::size_t t = 0; t < numSteps && t < MaxSteps; ++t)
        {
            truePositions.push_back(state.at(0, 0));

            observations[t].at(0, 0) = state.at(0, 0) + measurementNoise(rng);

            StateVector noise{ { processNoise(rng) }, { processNoise(rng) } };
            state = A_ * state + noise;
        }
    }
}
