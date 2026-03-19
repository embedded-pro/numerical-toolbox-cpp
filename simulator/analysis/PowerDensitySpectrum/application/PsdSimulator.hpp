#pragma once

#include "simulator/utils/SignalGenerator.hpp"
#include <cstddef>
#include <vector>

namespace simulator::analysis::psd
{
    using utils::NoiseConfiguration;
    using utils::NoiseType;
    using utils::SignalComponent;

    enum class WindowType
    {
        Rectangular,
        Hamming,
        Hanning,
        Blackman
    };

    struct PsdResult
    {
        std::vector<float> time;
        std::vector<float> signal;
        std::vector<float> frequencies;
        std::vector<float> powerDensity;
        std::vector<float> powerDensityDb;
    };

    class PsdSimulator
    {
    public:
        struct Configuration
        {
            std::size_t inputSize = 1024;
            std::size_t segmentSize = 256;
            float sampleRateHz = 44100.0f;
            WindowType windowType = WindowType::Hamming;
            std::size_t overlapPercent = 50;
            NoiseConfiguration noise = { NoiseType::WhiteGaussian, 0.1f };
            std::vector<SignalComponent> signalComponents = {
                { 1000.0f, 0.15f },
                { 5000.0f, 0.5f },
                { 12000.0f, 0.25f }
            };
        };

        void Configure(const Configuration& config);
        PsdResult Compute();

        static std::vector<std::size_t> SupportedSegmentSizes();
        static std::vector<std::size_t> SupportedOverlapValues();

    private:
        Configuration configuration;
    };
}
