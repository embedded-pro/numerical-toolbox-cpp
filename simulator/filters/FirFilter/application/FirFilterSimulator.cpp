#include "simulator/filters/FirFilter/application/FirFilterSimulator.hpp"
#include "numerical/analysis/FastFourierTransformRadix2Impl.hpp"
#include "simulator/utils/TwiddleFactorsTable.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace simulator::filters::fir
{
    namespace
    {
        std::vector<float> WindowedSinc(std::size_t order, float normalizedCutoff)
        {
            std::vector<float> coeffs(order);
            auto mid = static_cast<float>(order - 1) / 2.0f;

            for (std::size_t i = 0; i < order; ++i)
            {
                auto n = static_cast<float>(i) - mid;

                float sinc;
                if (std::abs(n) < 1e-6f)
                    sinc = 2.0f * normalizedCutoff;
                else
                    sinc = std::sin(2.0f * std::numbers::pi_v<float> * normalizedCutoff * n) / (std::numbers::pi_v<float> * n);

                auto hamming = 0.54f - 0.46f * std::cos(2.0f * std::numbers::pi_v<float> * static_cast<float>(i) / static_cast<float>(order - 1));
                coeffs[i] = sinc * hamming;
            }

            return coeffs;
        }

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

        std::vector<float> ApplyFilter(const std::vector<float>& coefficients, const std::vector<float>& input)
        {
            std::vector<float> output(input.size());

            for (std::size_t n = 0; n < input.size(); ++n)
            {
                float sum = 0.0f;
                for (std::size_t k = 0; k < coefficients.size(); ++k)
                {
                    if (n >= k)
                        sum += coefficients[k] * input[n - k];
                }
                output[n] = sum;
            }

            return output;
        }

        template<std::size_t N>
        SimulationResult ComputeWithFftSize(const std::vector<float>& inputSignal,
            const std::vector<float>& outputSignal, const std::vector<float>& coefficients,
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

            result.impulseResponse = coefficients;
            result.impulseSampleIndex.resize(coefficients.size());
            for (std::size_t i = 0; i < coefficients.size(); ++i)
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

    void FirFilterSimulator::Configure(const Configuration& config)
    {
        configuration = config;
    }

    std::vector<float> FirFilterSimulator::DesignCoefficients(const FilterDesignConfig& design)
    {
        auto normalizedCutoff = design.cutoffHz / design.sampleRateHz;

        switch (design.type)
        {
            case FilterType::LowPass:
                return WindowedSinc(design.order, normalizedCutoff);

            case FilterType::HighPass:
            {
                auto lowpass = WindowedSinc(design.order, normalizedCutoff);
                auto mid = (design.order - 1) / 2;
                for (std::size_t i = 0; i < lowpass.size(); ++i)
                    lowpass[i] = -lowpass[i];
                lowpass[mid] += 1.0f;
                return lowpass;
            }

            case FilterType::BandPass:
            {
                auto normalizedLow = design.cutoffHz / design.sampleRateHz;
                auto normalizedHigh = design.cutoffHighHz / design.sampleRateHz;
                auto highpass = WindowedSinc(design.order, normalizedHigh);
                auto lowpass = WindowedSinc(design.order, normalizedLow);

                auto mid = (design.order - 1) / 2;
                for (std::size_t i = 0; i < lowpass.size(); ++i)
                    lowpass[i] = -lowpass[i];
                lowpass[mid] += 1.0f;

                std::vector<float> bandpass(design.order);
                for (std::size_t i = 0; i < design.order; ++i)
                    bandpass[i] = highpass[i] - lowpass[i];
                return bandpass;
            }

            default:
                return WindowedSinc(design.order, normalizedCutoff);
        }
    }

    SimulationResult FirFilterSimulator::Run()
    {
        auto coefficients = DesignCoefficients(configuration.filter);

        auto numSamples = static_cast<std::size_t>(configuration.simulation.duration * configuration.simulation.sampleRateHz);
        auto inputSignal = utils::SignalGenerator::Generate(numSamples, configuration.simulation.sampleRateHz,
            configuration.simulation.signalComponents);

        auto outputSignal = ApplyFilter(coefficients, inputSignal);

        auto fftSize = NextPowerOfTwo(numSamples);

        switch (fftSize)
        {
            case 64:
                return ComputeWithFftSize<64>(inputSignal, outputSignal, coefficients, configuration.simulation.sampleRateHz);
            case 128:
                return ComputeWithFftSize<128>(inputSignal, outputSignal, coefficients, configuration.simulation.sampleRateHz);
            case 256:
                return ComputeWithFftSize<256>(inputSignal, outputSignal, coefficients, configuration.simulation.sampleRateHz);
            case 512:
                return ComputeWithFftSize<512>(inputSignal, outputSignal, coefficients, configuration.simulation.sampleRateHz);
            case 1024:
                return ComputeWithFftSize<1024>(inputSignal, outputSignal, coefficients, configuration.simulation.sampleRateHz);
            case 2048:
                return ComputeWithFftSize<2048>(inputSignal, outputSignal, coefficients, configuration.simulation.sampleRateHz);
            case 4096:
            default:
                return ComputeWithFftSize<4096>(inputSignal, outputSignal, coefficients, configuration.simulation.sampleRateHz);
        }
    }
}
