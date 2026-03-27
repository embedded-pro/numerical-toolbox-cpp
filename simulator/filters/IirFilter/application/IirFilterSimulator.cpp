#include "simulator/filters/IirFilter/application/IirFilterSimulator.hpp"
#include "numerical/analysis/FastFourierTransformRadix2Impl.hpp"
#include "simulator/utils/TwiddleFactorsTable.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace simulator::filters::iir
{
    namespace
    {
        template<std::size_t N>
        std::vector<float> ComputeMagnitudeDb(const std::vector<float>& signal, float sampleRateHz, std::vector<float>& freqsOut)
        {
            utils::TwiddleFactorsTable<float, N / 2> twiddleFactors;
            ::analysis::FastFourierTransformRadix2Impl<float, N> fft(twiddleFactors);

            infra::BoundedVector<float>::WithMaxSize<N> input;
            for (std::size_t i = 0; i < std::min(signal.size(), N); ++i)
                input.push_back(signal[i]);
            while (input.size() < N)
                input.push_back(0.0f);

            auto& frequencyDomain = fft.Forward(input);

            freqsOut.resize(N / 2);
            std::vector<float> magnitudeDb(N / 2);

            for (std::size_t i = 0; i < N / 2; ++i)
            {
                freqsOut[i] = static_cast<float>(i) * sampleRateHz / static_cast<float>(N);
                float real = frequencyDomain[i].Real();
                float imag = frequencyDomain[i].Imaginary();
                float mag = 2.0f * std::sqrt(real * real + imag * imag) / static_cast<float>(N);
                magnitudeDb[i] = 20.0f * std::log10(std::max(mag, 1e-10f));
            }

            return magnitudeDb;
        }

        std::vector<float> ApplyBiquad(const BiquadCoefficients& bq, const std::vector<float>& input)
        {
            std::vector<float> output(input.size());
            float x1 = 0.0f, x2 = 0.0f, y1 = 0.0f, y2 = 0.0f;

            for (std::size_t n = 0; n < input.size(); ++n)
            {
                float x0 = input[n];
                output[n] = bq.b0 * x0 + bq.b1 * x1 + bq.b2 * x2 + bq.a1 * y1 + bq.a2 * y2;
                x2 = x1;
                x1 = x0;
                y2 = y1;
                y1 = output[n];
            }

            return output;
        }

        std::vector<float> ComputeImpulseResponse(const BiquadCoefficients& bq, std::size_t length)
        {
            std::vector<float> impulse(length, 0.0f);
            impulse[0] = 1.0f;
            return ApplyBiquad(bq, impulse);
        }

        template<std::size_t N>
        SimulationResult ComputeWithFftSize(const std::vector<float>& inputSignal,
            const std::vector<float>& outputSignal, const std::vector<float>& impulseResponse,
            float sampleRateHz)
        {
            SimulationResult result;

            result.time.resize(inputSignal.size());
            for (std::size_t i = 0; i < inputSignal.size(); ++i)
                result.time[i] = static_cast<float>(i) / sampleRateHz;

            result.inputSignal = inputSignal;
            result.outputSignal = outputSignal;

            result.inputMagnitudeDb = ComputeMagnitudeDb<N>(inputSignal, sampleRateHz, result.frequencies);

            std::vector<float> freqsUnused;
            result.outputMagnitudeDb = ComputeMagnitudeDb<N>(outputSignal, sampleRateHz, freqsUnused);

            result.impulseResponse = impulseResponse;
            result.impulseSampleIndex.resize(impulseResponse.size());
            for (std::size_t i = 0; i < impulseResponse.size(); ++i)
                result.impulseSampleIndex[i] = static_cast<float>(i);

            return result;
        }

        std::size_t NextPowerOfTwo(std::size_t n)
        {
            std::size_t power = 64;
            while (power < n)
                power *= 2;
            return std::min(power, std::size_t(4096));
        }
    }

    void IirFilterSimulator::Configure(const Configuration& config)
    {
        configuration = config;
    }

    BiquadCoefficients IirFilterSimulator::DesignBiquad(const FilterDesignConfig& design)
    {
        auto w0 = 2.0f * std::numbers::pi_v<float> * design.cutoffHz / design.sampleRateHz;
        auto cosW0 = std::cos(w0);
        auto sinW0 = std::sin(w0);
        auto alpha = sinW0 / (2.0f * design.qualityFactor);

        BiquadCoefficients bq;

        switch (design.type)
        {
            case FilterType::LowPass:
            {
                auto a0 = 1.0f + alpha;
                bq.b0 = ((1.0f - cosW0) / 2.0f) / a0;
                bq.b1 = (1.0f - cosW0) / a0;
                bq.b2 = ((1.0f - cosW0) / 2.0f) / a0;
                bq.a1 = (-2.0f * cosW0) / -a0;
                bq.a2 = (1.0f - alpha) / -a0;
                break;
            }

            case FilterType::HighPass:
            {
                auto a0 = 1.0f + alpha;
                bq.b0 = ((1.0f + cosW0) / 2.0f) / a0;
                bq.b1 = -(1.0f + cosW0) / a0;
                bq.b2 = ((1.0f + cosW0) / 2.0f) / a0;
                bq.a1 = (-2.0f * cosW0) / -a0;
                bq.a2 = (1.0f - alpha) / -a0;
                break;
            }

            case FilterType::BandPass:
            {
                auto a0 = 1.0f + alpha;
                bq.b0 = alpha / a0;
                bq.b1 = 0.0f;
                bq.b2 = -alpha / a0;
                bq.a1 = (-2.0f * cosW0) / -a0;
                bq.a2 = (1.0f - alpha) / -a0;
                break;
            }
        }

        return bq;
    }

    SimulationResult IirFilterSimulator::Run()
    {
        auto bq = DesignBiquad(configuration.filter);

        auto numSamples = static_cast<std::size_t>(configuration.simulation.duration * configuration.simulation.sampleRateHz);
        auto inputSignal = utils::SignalGenerator::Generate(numSamples, configuration.simulation.sampleRateHz,
            configuration.simulation.signalComponents);

        auto outputSignal = ApplyBiquad(bq, inputSignal);
        auto impulseResponse = ComputeImpulseResponse(bq, 64);

        auto fftSize = NextPowerOfTwo(numSamples);

        switch (fftSize)
        {
            case 64:
                return ComputeWithFftSize<64>(inputSignal, outputSignal, impulseResponse, configuration.simulation.sampleRateHz);
            case 128:
                return ComputeWithFftSize<128>(inputSignal, outputSignal, impulseResponse, configuration.simulation.sampleRateHz);
            case 256:
                return ComputeWithFftSize<256>(inputSignal, outputSignal, impulseResponse, configuration.simulation.sampleRateHz);
            case 512:
                return ComputeWithFftSize<512>(inputSignal, outputSignal, impulseResponse, configuration.simulation.sampleRateHz);
            case 1024:
                return ComputeWithFftSize<1024>(inputSignal, outputSignal, impulseResponse, configuration.simulation.sampleRateHz);
            case 2048:
                return ComputeWithFftSize<2048>(inputSignal, outputSignal, impulseResponse, configuration.simulation.sampleRateHz);
            case 4096:
            default:
                return ComputeWithFftSize<4096>(inputSignal, outputSignal, impulseResponse, configuration.simulation.sampleRateHz);
        }
    }
}
