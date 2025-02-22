#include "numerical/analysis/FastFourierTransformRadix2Impl.hpp"
#include "numerical/analysis/PowerDensitySpectrum.hpp"
#include "numerical/windowing/Windowing.hpp"
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
            const auto inputSize = Length;
            factors.reserve(inputSize);

            for (std::size_t k = 0; k < inputSize; ++k)
            {
                float angle = -M_PI * static_cast<float>(k) / static_cast<float>(inputSize);
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
    constexpr std::size_t inputSize = 1024;
    constexpr std::size_t fftLength = 256;
    const float sampleRate = 44100.0f;
    const float samplingTime = static_cast<float>(inputSize) / sampleRate;
    const float frequencyResolution = sampleRate / static_cast<float>(fftLength);

    auto signal = SignalGenerator<float>::GenerateSignal(inputSize, sampleRate);

    windowing::HammingWindow<float> window;
    using FFT = analysis::FastFourierTransformRadix2Impl<float, fftLength>;
    using TwiddleFactors = TwiddleFactorsImpl<float, fftLength / 2>;

    analysis::PowerSpectralDensity<float, fftLength, FFT, TwiddleFactors, 50> powerDensitySpectrum(window, samplingTime);

    infra::BoundedVector<float>::WithMaxSize<inputSize> input;
    for (const auto& value : signal)
        input.push_back(value);

    auto& magnitudes = powerDensitySpectrum.Calculate(input);

    std::vector<float> freqPoints;
    std::vector<float> magPoints;

    for (std::size_t i = 0; i < magnitudes.size(); ++i)
    {
        freqPoints.push_back(static_cast<float>(i) * frequencyResolution);
        magPoints.push_back(10 * std::log10(magnitudes[i]));
    }

    Plot psdPlot;
    psdPlot.xlabel("Frequency (Hz)");
    psdPlot.ylabel("Power/Frequency (dB/Hz)");
    psdPlot.grid().show();
    psdPlot.legend().show();
    psdPlot.drawCurve(freqPoints, magPoints).label("Power Spectral Density");

    Figure figure = { { psdPlot } };
    figure.title("Power Spectral Density Welch - Analysis");
    figure.size(800, 600);
    figure.save("Power Spectral Densit Welch - Analysis.pdf");

    return 0;
}
