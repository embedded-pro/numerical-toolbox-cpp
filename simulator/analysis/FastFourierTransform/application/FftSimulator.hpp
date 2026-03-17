#pragma once

#include "simulator/analysis/FastFourierTransform/application/SignalGenerator.hpp"
#include <cstddef>
#include <vector>

namespace simulator::analysis
{
    struct FftResult
    {
        std::vector<float> frequencies;
        std::vector<float> magnitudes;
    };

    class FftSimulator
    {
    public:
        struct Configuration
        {
            std::size_t fftSize = 1024;
            float sampleRateHz = 44100.0f;
            std::vector<SignalComponent> signalComponents = {
                { 1000.0f, 0.15f },
                { 5000.0f, 0.5f },
                { 12000.0f, 0.25f }
            };
        };

        void Configure(const Configuration& config);
        FftResult Compute();

        static std::vector<std::size_t> SupportedFftSizes();

    private:
        Configuration configuration;
    };
}
