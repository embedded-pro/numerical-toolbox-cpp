#include "simulator/analysis/FastFourierTransform/application/FftSimulator.hpp"
#include "numerical/analysis/FastFourierTransformRadix2Impl.hpp"
#include "numerical/windowing/Windowing.hpp"
#include "simulator/utils/TwiddleFactorsTable.hpp"
#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

namespace simulator::analysis
{
    namespace
    {
        std::vector<float> ApplyWindow(const std::vector<float>& signal, WindowType windowType)
        {
            std::unique_ptr<windowing::Window<float>> window;

            switch (windowType)
            {
                case WindowType::Hamming:
                    window = std::make_unique<windowing::HammingWindow<float>>();
                    break;
                case WindowType::Hanning:
                    window = std::make_unique<windowing::HanningWindow<float>>();
                    break;
                case WindowType::Blackman:
                    window = std::make_unique<windowing::BlackmanWindow<float>>();
                    break;
                case WindowType::Rectangular:
                default:
                    window = std::make_unique<windowing::RectangularWindow<float>>();
                    break;
            }

            std::vector<float> windowed(signal.size());
            for (std::size_t i = 0; i < signal.size(); ++i)
                windowed[i] = signal[i] * (*window)(i, signal.size());

            return windowed;
        }

        template<std::size_t N>
        FftResult ComputeForSize(float sampleRateHz, const std::vector<float>& signal, const std::vector<float>& windowedSignal)
        {
            utils::TwiddleFactorsTable<float, N / 2> twiddleFactors;
            ::analysis::FastFourierTransformRadix2Impl<float, N> fft(twiddleFactors);

            infra::BoundedVector<float>::WithMaxSize<N> input;
            for (std::size_t i = 0; i < std::min(windowedSignal.size(), N); ++i)
                input.push_back(windowedSignal[i]);

            auto& frequencyDomain = fft.Forward(input);

            FftResult result;
            result.sampleRateHz = sampleRateHz;

            result.time.resize(N);
            result.signal.resize(N);
            result.windowedSignal.resize(N);
            for (std::size_t i = 0; i < N; ++i)
            {
                result.time[i] = static_cast<float>(i) / sampleRateHz;
                result.signal[i] = i < signal.size() ? signal[i] : 0.0f;
                result.windowedSignal[i] = i < windowedSignal.size() ? windowedSignal[i] : 0.0f;
            }

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

        auto signal = utils::SignalGenerator::Generate(configuration.fftSize, configuration.sampleRateHz, configuration.signalComponents);
        auto windowedSignal = ApplyWindow(signal, configuration.windowType);

        switch (configuration.fftSize)
        {
            case 64:
                return ComputeForSize<64>(configuration.sampleRateHz, signal, windowedSignal);
            case 128:
                return ComputeForSize<128>(configuration.sampleRateHz, signal, windowedSignal);
            case 256:
                return ComputeForSize<256>(configuration.sampleRateHz, signal, windowedSignal);
            case 512:
                return ComputeForSize<512>(configuration.sampleRateHz, signal, windowedSignal);
            case 1024:
                return ComputeForSize<1024>(configuration.sampleRateHz, signal, windowedSignal);
            case 2048:
                return ComputeForSize<2048>(configuration.sampleRateHz, signal, windowedSignal);
            case 4096:
                return ComputeForSize<4096>(configuration.sampleRateHz, signal, windowedSignal);
            default:
                throw std::invalid_argument("Unsupported FFT size. Must be a power of 2 between 64 and 4096.");
        }
    }

    std::vector<std::size_t> FftSimulator::SupportedFftSizes()
    {
        return { 64, 128, 256, 512, 1024, 2048, 4096 };
    }
}
