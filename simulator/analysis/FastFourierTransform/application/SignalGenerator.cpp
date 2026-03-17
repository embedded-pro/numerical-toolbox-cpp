#include "simulator/analysis/FastFourierTransform/application/SignalGenerator.hpp"
#include <cmath>
#include <numbers>

namespace simulator::analysis
{
    std::vector<float> SignalGenerator::Generate(std::size_t length, float sampleRateHz, const std::vector<SignalComponent>& components)
    {
        std::vector<float> signal(length, 0.0f);

        for (std::size_t i = 0; i < length; ++i)
        {
            float t = static_cast<float>(i) / sampleRateHz;

            for (const auto& component : components)
                signal[i] += component.amplitude * std::sin(2.0f * std::numbers::pi_v<float> * component.frequencyHz * t);
        }

        return signal;
    }
}
