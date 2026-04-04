#include "simulator/controllers/BayesianMpcCalibration/application/EmKfPipeline.hpp"
#include <cmath>
#include <limits>

namespace simulator::controllers
{
    EmKfResult EmKfPipeline::Run(
        const std::array<MeasurementVector, MaxSteps>& observations,
        std::size_t numSteps,
        const EmParameters& initialGuess,
        const EmKfConfig& config)
    {
        EmKfResult result{};
        result.logLikelihoodHistory.reserve(config.maxEmIterations);

        EmParameters currentParams = initialGuess;
        float prevLogLikelihood = std::numeric_limits<float>::lowest();

        for (std::size_t iter = 0; iter < config.maxEmIterations; ++iter)
        {
            const auto stepResult = em_.Run(observations, numSteps, currentParams, 1,
                std::numeric_limits<float>::lowest());

            result.logLikelihoodHistory.push_back(stepResult.logLikelihood);
            currentParams = stepResult.parameters;

            if (std::abs(stepResult.logLikelihood - prevLogLikelihood) < config.convergenceTolerance)
            {
                result.emResult = stepResult;
                result.emResult.iterations = iter + 1;
                result.emResult.converged = true;
                return result;
            }

            prevLogLikelihood = stepResult.logLikelihood;
        }

        result.emResult.parameters = currentParams;
        result.emResult.logLikelihood = prevLogLikelihood;
        result.emResult.iterations = config.maxEmIterations;
        result.emResult.converged = false;
        return result;
    }
}
