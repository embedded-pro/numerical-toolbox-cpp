#include "simulator/utils/SignalGenerator.hpp"
#include <cmath>
#include <numbers>
#include <random>

namespace simulator::utils
{
    std::vector<float> SignalGenerator::Generate(std::size_t length, float sampleRateHz,
        const std::vector<SignalComponent>& components, const NoiseConfiguration& noise)
    {
        std::vector<float> signal(length, 0.0f);

        for (std::size_t i = 0; i < length; ++i)
        {
            float t = static_cast<float>(i) / sampleRateHz;

            for (const auto& component : components)
                signal[i] += component.amplitude * std::sin(2.0f * std::numbers::pi_v<float> * component.frequencyHz * t);
        }

        if (noise.type == NoiseType::WhiteGaussian && noise.amplitude > 0.0f)
        {
            std::mt19937 generator(42);
            std::normal_distribution<float> distribution(0.0f, noise.amplitude);

            for (std::size_t i = 0; i < length; ++i)
                signal[i] += distribution(generator);
        }

        return signal;
    }
}
