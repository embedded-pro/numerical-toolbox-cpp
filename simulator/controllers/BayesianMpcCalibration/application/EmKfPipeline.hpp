#pragma once

#include "numerical/estimators/offline/ExpectationMaximization.hpp"
#include "numerical/math/Matrix.hpp"
#include <array>
#include <cstddef>
#include <vector>

namespace simulator::controllers
{
    struct EmKfConfig
    {
        std::size_t maxEmIterations = 50;
        float convergenceTolerance = 1e-4f;
    };

    struct EmKfResult
    {
        estimators::ExpectationMaximization<2, 1, 200>::EmResult emResult;
        std::vector<float> logLikelihoodHistory;
    };

    class EmKfPipeline
    {
    public:
        static constexpr std::size_t MaxSteps = 200;
        using MeasurementVector = math::Vector<float, 1>;
        using EmType = estimators::ExpectationMaximization<2, 1, MaxSteps>;
        using EmParameters = EmType::EmParameters;

        EmKfResult Run(
            const std::array<MeasurementVector, MaxSteps>& observations,
            std::size_t numSteps,
            const EmParameters& initialGuess,
            const EmKfConfig& config);

    private:
        EmType em_;
    };
}
