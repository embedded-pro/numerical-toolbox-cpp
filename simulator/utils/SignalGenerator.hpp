#pragma once

#include <cstddef>
#include <vector>

namespace simulator::utils
{
    struct SignalComponent
    {
        float frequencyHz;
        float amplitude;
    };

    enum class NoiseType
    {
        None,
        WhiteGaussian
    };

    struct NoiseConfiguration
    {
        NoiseType type = NoiseType::None;
        float amplitude = 0.0f;
    };

    class SignalGenerator
    {
    public:
        static std::vector<float> Generate(std::size_t length, float sampleRateHz,
            const std::vector<SignalComponent>& components,
            const NoiseConfiguration& noise = NoiseConfiguration{});
    };
}
