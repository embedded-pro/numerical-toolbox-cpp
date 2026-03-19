#include "simulator/analysis/PowerDensitySpectrum/application/PsdSimulator.hpp"
#include "numerical/analysis/FastFourierTransformRadix2Impl.hpp"
#include "numerical/analysis/PowerDensitySpectrum.hpp"
#include "numerical/windowing/Windowing.hpp"
#include "simulator/utils/TwiddleFactorsTable.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace simulator::analysis::psd
{
    namespace
    {
        template<std::size_t SegmentSize, std::size_t Overlap>
        PsdResult ComputeForSizeAndOverlap(float sampleRateHz, std::size_t inputSize,
            const std::vector<float>& signal, windowing::Window<float>& window)
        {
            using FFT = ::analysis::FastFourierTransformRadix2Impl<float, SegmentSize>;
            using TwiddleFactors = utils::TwiddleFactorsTable<float, SegmentSize / 2>;

            float samplingTime = static_cast<float>(inputSize) / sampleRateHz;
            float frequencyResolution = sampleRateHz / static_cast<float>(SegmentSize);

            ::analysis::PowerSpectralDensity<float, SegmentSize, FFT, TwiddleFactors, Overlap> psd(window, samplingTime);

            infra::BoundedVector<float>::WithMaxSize<4096> input;
            for (std::size_t i = 0; i < std::min(signal.size(), static_cast<std::size_t>(4096)); ++i)
                input.push_back(signal[i]);

            auto& magnitudes = psd.Calculate(input);

            PsdResult result;
            result.sampleRateHz = sampleRateHz;

            result.time.resize(inputSize);
            result.signal.resize(inputSize);
            for (std::size_t i = 0; i < inputSize; ++i)
            {
                result.time[i] = static_cast<float>(i) / sampleRateHz;
                result.signal[i] = i < signal.size() ? signal[i] : 0.0f;
            }

            result.frequencies.resize(magnitudes.size());
            result.powerDensity.resize(magnitudes.size());
            result.powerDensityDb.resize(magnitudes.size());

            for (std::size_t i = 0; i < magnitudes.size(); ++i)
            {
                result.frequencies[i] = static_cast<float>(i) * frequencyResolution;
                result.powerDensity[i] = magnitudes[i];
                result.powerDensityDb[i] = magnitudes[i] > 0.0f ? 10.0f * std::log10(magnitudes[i]) : -100.0f;
            }

            return result;
        }

        template<std::size_t SegmentSize>
        PsdResult DispatchOverlap(std::size_t overlapPercent, float sampleRateHz, std::size_t inputSize,
            const std::vector<float>& signal, windowing::Window<float>& window)
        {
            switch (overlapPercent)
            {
                case 0:
                    return ComputeForSizeAndOverlap<SegmentSize, 0>(sampleRateHz, inputSize, signal, window);
                case 25:
                    return ComputeForSizeAndOverlap<SegmentSize, 25>(sampleRateHz, inputSize, signal, window);
                case 50:
                    return ComputeForSizeAndOverlap<SegmentSize, 50>(sampleRateHz, inputSize, signal, window);
                case 75:
                    return ComputeForSizeAndOverlap<SegmentSize, 75>(sampleRateHz, inputSize, signal, window);
                default:
                    throw std::invalid_argument("Unsupported overlap. Must be 0, 25, 50, or 75.");
            }
        }
    }

    void PsdSimulator::Configure(const Configuration& config)
    {
        configuration = config;
    }

    PsdResult PsdSimulator::Compute()
    {
        if (configuration.sampleRateHz <= 0.0f)
            throw std::invalid_argument("Sample rate must be greater than zero.");

        if (configuration.inputSize < configuration.segmentSize)
            throw std::invalid_argument("Input size must be greater than or equal to segment size.");

        auto signal = utils::SignalGenerator::Generate(configuration.inputSize, configuration.sampleRateHz,
            configuration.signalComponents, configuration.noise);

        std::unique_ptr<windowing::Window<float>> window;
        switch (configuration.windowType)
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

        switch (configuration.segmentSize)
        {
            case 64:
                return DispatchOverlap<64>(configuration.overlapPercent, configuration.sampleRateHz, configuration.inputSize, signal, *window);
            case 128:
                return DispatchOverlap<128>(configuration.overlapPercent, configuration.sampleRateHz, configuration.inputSize, signal, *window);
            case 256:
                return DispatchOverlap<256>(configuration.overlapPercent, configuration.sampleRateHz, configuration.inputSize, signal, *window);
            case 512:
                return DispatchOverlap<512>(configuration.overlapPercent, configuration.sampleRateHz, configuration.inputSize, signal, *window);
            case 1024:
                return DispatchOverlap<1024>(configuration.overlapPercent, configuration.sampleRateHz, configuration.inputSize, signal, *window);
            default:
                throw std::invalid_argument("Unsupported segment size. Must be a power of 2 between 64 and 1024.");
        }
    }

    std::vector<std::size_t> PsdSimulator::SupportedSegmentSizes()
    {
        return { 64, 128, 256, 512, 1024 };
    }

    std::vector<std::size_t> PsdSimulator::SupportedOverlapValues()
    {
        return { 0, 25, 50, 75 };
    }
}
