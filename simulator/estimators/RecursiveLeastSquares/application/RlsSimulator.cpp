#include "simulator/estimators/RecursiveLeastSquares/application/RlsSimulator.hpp"
#include "numerical/estimators/online/RecursiveLeastSquares.hpp"
#include <cmath>
#include <random>

namespace simulator::estimators::rls
{
    namespace
    {
        template<std::size_t Features>
        RlsResult RunEstimation(const RlsConfig& config)
        {
            ::estimators::RecursiveLeastSquares<float, Features> estimator(config.initialCovariance, config.forgettingFactor);

            RlsResult result;
            result.sampleIndex.resize(config.numSamples);
            result.trueOutput.resize(config.numSamples);
            result.estimatedOutput.resize(config.numSamples);
            result.coefficientHistory.resize(Features);
            for (auto& history : result.coefficientHistory)
                history.resize(config.numSamples);
            result.innovationHistory.resize(config.numSamples);
            result.uncertaintyHistory.resize(config.numSamples);

            std::mt19937 rng(42);
            std::normal_distribution<float> noise(0.0f, config.noiseAmplitude);
            std::uniform_real_distribution<float> input(-1.0f, 1.0f);

            for (std::size_t n = 0; n < config.numSamples; ++n)
            {
                result.sampleIndex[n] = static_cast<float>(n);

                math::Matrix<float, Features, 1> regressor;
                regressor.at(0, 0) = 1.0f;
                for (std::size_t f = 1; f < Features; ++f)
                    regressor.at(f, 0) = input(rng);

                float trueY = 0.0f;
                for (std::size_t f = 0; f < Features && f < config.trueCoefficients.size(); ++f)
                    trueY += config.trueCoefficients[f] * regressor.at(f, 0);

                float noisyY = trueY + noise(rng);
                result.trueOutput[n] = trueY;

                math::Matrix<float, 1, 1> y;
                y.at(0, 0) = noisyY;

                auto metrics = estimator.Update(regressor, y);

                auto estimated = 0.0f;
                const auto& theta = estimator.Coefficients();
                for (std::size_t f = 0; f < Features; ++f)
                {
                    result.coefficientHistory[f][n] = theta.at(f, 0);
                    estimated += theta.at(f, 0) * regressor.at(f, 0);
                }
                result.estimatedOutput[n] = estimated;
                result.innovationHistory[n] = metrics.innovation;
                result.uncertaintyHistory[n] = metrics.uncertainty;
            }

            return result;
        }
    }

    void RlsSimulator::Configure(const Configuration& config)
    {
        configuration = config;
    }

    RlsResult RlsSimulator::Run()
    {
        const auto features = configuration.rls.trueCoefficients.size();

        switch (features)
        {
        case 2:
            return RunEstimation<2>(configuration.rls);
        case 3:
            return RunEstimation<3>(configuration.rls);
        default:
            return RunEstimation<3>(configuration.rls);
        }
    }
}
