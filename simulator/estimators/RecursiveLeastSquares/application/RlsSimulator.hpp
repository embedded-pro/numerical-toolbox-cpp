#pragma once

#include <cstddef>
#include <vector>

namespace simulator::estimators::rls
{
    struct RlsConfig
    {
        float forgettingFactor = 0.99f;
        float initialCovariance = 1000.0f;
        std::size_t numSamples = 200;
        float noiseAmplitude = 0.1f;
        std::vector<float> trueCoefficients = { 2.0f, -1.5f, 0.8f };
    };

    struct RlsResult
    {
        std::vector<float> sampleIndex;
        std::vector<float> trueOutput;
        std::vector<float> estimatedOutput;
        std::vector<std::vector<float>> coefficientHistory;
        std::vector<float> innovationHistory;
        std::vector<float> uncertaintyHistory;
    };

    class RlsSimulator
    {
    public:
        struct Configuration
        {
            RlsConfig rls;
        };

        void Configure(const Configuration& config);
        RlsResult Run();

    private:
        Configuration configuration;
    };
}
