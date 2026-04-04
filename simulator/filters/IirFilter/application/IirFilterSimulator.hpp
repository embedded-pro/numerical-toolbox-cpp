#pragma once

#include "simulator/utils/SignalGenerator.hpp"
#include <cstddef>
#include <vector>

namespace simulator::filters::iir
{
    using utils::SignalComponent;

    enum class FilterType
    {
        LowPass,
        HighPass,
        BandPass
    };

    struct BiquadCoefficients
    {
        float b0 = 0.0f;
        float b1 = 0.0f;
        float b2 = 0.0f;
        float a1 = 0.0f;
        float a2 = 0.0f;
    };

    struct FilterDesignConfig
    {
        FilterType type = FilterType::LowPass;
        float cutoffHz = 1000.0f;
        float sampleRateHz = 8000.0f;
        float qualityFactor = 0.707f;
    };

    struct SimulationConfig
    {
        float duration = 0.05f;
        float sampleRateHz = 8000.0f;
        std::vector<SignalComponent> signalComponents = {
            { 200.0f, 1.0f },
            { 2000.0f, 0.5f },
        };
    };

    struct SimulationResult
    {
        std::vector<float> time;
        std::vector<float> inputSignal;
        std::vector<float> outputSignal;
        std::vector<float> frequencies;
        std::vector<float> inputMagnitudeDb;
        std::vector<float> outputMagnitudeDb;
        std::vector<float> impulseResponse;
        std::vector<float> impulseSampleIndex;
    };

    class IirFilterSimulator
    {
    public:
        struct Configuration
        {
            FilterDesignConfig filter;
            SimulationConfig simulation;
        };

        void Configure(const Configuration& config);
        SimulationResult Run();

        static BiquadCoefficients DesignBiquad(const FilterDesignConfig& design);

    private:
        Configuration configuration;
    };
}
