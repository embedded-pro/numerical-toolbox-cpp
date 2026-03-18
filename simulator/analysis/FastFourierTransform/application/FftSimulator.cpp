#include "simulator/analysis/FastFourierTransform/application/FftSimulator.hpp"
#include "numerical/analysis/FastFourierTransformRadix2Impl.hpp"
#include "simulator/analysis/FastFourierTransform/application/TwiddleFactorsTable.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace simulator::analysis
{
    namespace
    {
        template<std::size_t N>
        FftResult ComputeForSize(float sampleRateHz, const std::vector<float>& signal)
        {
            TwiddleFactorsTable<float, N / 2> twiddleFactors;
            ::analysis::FastFourierTransformRadix2Impl<float, N> fft(twiddleFactors);

            infra::BoundedVector<float>::WithMaxSize<N> input;
            for (std::size_t i = 0; i < std::min(signal.size(), N); ++i)
                input.push_back(signal[i]);

            auto& frequencyDomain = fft.Forward(input);

            FftResult result;
            result.frequencies.resize(N / 2);
            result.magnitudes.resize(N / 2);

            for (std::size_t i = 0; i < N / 2; ++i)
            {
                result.frequencies[i] = static_cast<float>(i) * sampleRateHz / static_cast<float>(N);
                float real = frequencyDomain[i].Real();
                float imag = frequencyDomain[i].Imaginary();
                result.magnitudes[i] = 2.0f * std::sqrt(real * real + imag * imag) / static_cast<float>(N);
            }

            return result;
        }
    }

    void FftSimulator::Configure(const Configuration& config)
    {
        configuration = config;
    }

    FftResult FftSimulator::Compute()
    {
        if (configuration.sampleRateHz <= 0.0f)
            throw std::invalid_argument("Sample rate must be greater than zero.");

        auto signal = SignalGenerator::Generate(configuration.fftSize, configuration.sampleRateHz, configuration.signalComponents);

        switch (configuration.fftSize)
        {
            case 64:
                return ComputeForSize<64>(configuration.sampleRateHz, signal);
            case 128:
                return ComputeForSize<128>(configuration.sampleRateHz, signal);
            case 256:
                return ComputeForSize<256>(configuration.sampleRateHz, signal);
            case 512:
                return ComputeForSize<512>(configuration.sampleRateHz, signal);
            case 1024:
                return ComputeForSize<1024>(configuration.sampleRateHz, signal);
            case 2048:
                return ComputeForSize<2048>(configuration.sampleRateHz, signal);
            case 4096:
                return ComputeForSize<4096>(configuration.sampleRateHz, signal);
            default:
                throw std::invalid_argument("Unsupported FFT size. Must be a power of 2 between 64 and 4096.");
        }
    }

    std::vector<std::size_t> FftSimulator::SupportedFftSizes()
    {
        return { 64, 128, 256, 512, 1024, 2048, 4096 };
    }
}
