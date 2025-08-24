#include "numerical/analysis/FastFourierTransform.hpp"
#include "numerical/analysis/FastFourierTransformRadix2Impl.hpp"
#include <cmath>
#include <iostream>
#include <sciplot/sciplot.hpp>
#include <vector>

using namespace sciplot;

namespace
{
    template<typename QNumberType, std::size_t Length>
    class TwiddleFactorsImpl
        : public analysis::TwiddleFactors<QNumberType, Length>
    {
    public:
        TwiddleFactorsImpl()
        {
            factors.reserve(Length);

            for (std::size_t k = 0; k < Length; ++k)
            {
                float angle = -M_PI * static_cast<float>(k) / static_cast<float>(Length);
                factors.emplace_back(std::cos(angle), std::sin(angle));
            }
        }

        math::Complex<QNumberType>& operator[](std::size_t n) override
        {
            return factors[n];
        }

    private:
        std::vector<math::Complex<QNumberType>> factors;
    };

    template<typename QNumberType>
    class SignalGenerator
    {
    public:
        static std::vector<QNumberType> GenerateSignal(std::size_t length, float sampleRate)
        {
            std::vector<QNumberType> signal(length);

            const float f1 = 1000.0f;
            const float f2 = 5000.0f;
            const float f3 = 12000.0f;

            for (std::size_t i = 0; i < length; ++i)
            {
                float t = static_cast<float>(i) / sampleRate;
                signal[i] = 0.15f * std::sin(2.0f * M_PI * f1 * t) +
                            0.5f * std::sin(2.0f * M_PI * f2 * t) +
                            0.25f * std::sin(2.0f * M_PI * f3 * t);
            }

            return signal;
        }
    };
}

int main()
{
    constexpr std::size_t N = 1024;
    const float sampleRate = 44100.0f;

    auto signal = SignalGenerator<float>::GenerateSignal(N, sampleRate);

    TwiddleFactorsImpl<float, N / 2> twiddleFactors;
    analysis::FastFourierTransformRadix2Impl<float, N> fft(twiddleFactors);

    infra::BoundedVector<float>::WithMaxSize<N> input;
    for (const auto& value : signal)
        input.push_back(value);

    auto& frequencyDomain = fft.Forward(input);

    std::vector<float> frequencies(N / 2);
    std::vector<float> magnitudes(N / 2);

    for (std::size_t i = 0; i < N / 2; ++i)
    {
        frequencies[i] = static_cast<float>(i) * sampleRate / N;
        auto complex = frequencyDomain[i];
        magnitudes[i] = std::sqrt(math::ToFloat(complex.Real()) * math::ToFloat(complex.Real()) +
                                  math::ToFloat(complex.Imaginary()) * math::ToFloat(complex.Imaginary()));
    }

    auto& reconstructedSignal = fft.Inverse(frequencyDomain);

    std::vector<float> timePoints(N);
    std::vector<float> originalSignalValues(signal.begin(), signal.end());
    std::vector<float> reconstructedSignalValues;

    for (std::size_t i = 0; i < N; ++i)
    {
        timePoints[i] = static_cast<float>(i) / sampleRate;
        reconstructedSignalValues.push_back(reconstructedSignal[i]);
    }

    float maxError = 0.0f;
    float totalError = 0.0f;
    for (std::size_t i = 0; i < N; ++i)
    {
        float error = std::abs(originalSignalValues[i] - reconstructedSignalValues[i]);
        maxError = std::max(maxError, error);
        totalError += error;
    }
    float avgError = totalError / N;
    std::cout << "Reconstruction Statistics:\n";
    std::cout << "Maximum Error: " << maxError << "\n";
    std::cout << "Average Error: " << avgError << "\n";

    Plot timePlot;
    timePlot.xlabel("Time (s)");
    timePlot.ylabel("Amplitude");
    timePlot.grid().show();
    timePlot.legend().show();
    timePlot.drawCurve(timePoints, originalSignalValues).label("Original Signal");
    timePlot.drawCurve(timePoints, reconstructedSignalValues).label("Reconstructed Signal");

    Plot freqPlot;
    freqPlot.xlabel("Frequency (Hz)");
    freqPlot.ylabel("Magnitude");
    freqPlot.grid().show();
    freqPlot.legend().show();
    freqPlot.drawCurve(frequencies, magnitudes).label("Frequency Spectrum");

    Figure figure = { { timePlot, freqPlot } };
    figure.title("Signal Analysis using FFT");
    figure.size(800, 600);
    figure.save("FFT Radix 2 - Analysis.pdf");

    return 0;
}
