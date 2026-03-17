#pragma once

#include <cstddef>
#include <vector>

namespace simulator::analysis
{
    struct SignalComponent
    {
        float frequencyHz;
        float amplitude;
    };

    class SignalGenerator
    {
    public:
        static std::vector<float> Generate(std::size_t length, float sampleRateHz, const std::vector<SignalComponent>& components);
    };
}
