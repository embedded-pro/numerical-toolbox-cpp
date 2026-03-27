#pragma once

#include "simulator/utils/SignalGenerator.hpp"
#include <cstddef>
#include <vector>

namespace simulator::filters::fir
{
    using utils::SignalComponent;

    enum class FilterType
    {
        LowPass,
        HighPass,
        BandPass
    };

    struct FilterDesignConfig
    {
        FilterType type = FilterType::LowPass;
        float cutoffHz = 1000.0f;
        float cutoffHighHz = 3000.0f;
        float sampleRateHz = 44100.0f;
        std::size_t order = 31;
    };

    struct SimulationConfig
    {
        float duration = 0.02f;
        float sampleRateHz = 44100.0f;
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

    class FirFilterSimulator
    {
    public:
        struct Configuration
        {
            FilterDesignConfig filter;
            SimulationConfig simulation;
        };

        void Configure(const Configuration& config);
        SimulationResult Run();

        static std::vector<float> DesignCoefficients(const FilterDesignConfig& design);

    private:
        Configuration configuration;
    };
}
