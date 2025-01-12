#include "analysis/FrequencyResponse.hpp"
#include <sciplot/sciplot.hpp>

using namespace sciplot;

namespace
{
    struct FilterCoeff
    {
        std::vector<float> b;
        std::vector<float> a;
        float sampleFrequency;
    };

    FilterCoeff DesignLowPass(float cutofFrequency, float sampleFrequency)
    {
        float w0 = 2.0f * M_PI * cutofFrequency / sampleFrequency;
        float alpha = std::sin(w0) / (2.0f * 0.707f); // Q = 0.707 for Butterworth response
        float cosw0 = std::cos(w0);
        float a0 = 1.0f + alpha;

        return FilterCoeff{
            { (1.0f - cosw0) / (2.0f * a0),
                (1.0f - cosw0) / a0,
                (1.0f - cosw0) / (2.0f * a0) },
            { 1.0f,
                -2.0f * cosw0 / a0,
                (1.0f - alpha) / a0 },
            sampleFrequency
        };
    }

    FilterCoeff DesignHighPass(float cutofFrequency, float sampleFrequency)
    {
        float w0 = 2.0f * M_PI * cutofFrequency / sampleFrequency;
        float alpha = std::sin(w0) / (2.0f * 0.707f); // Q = 0.707 for Butterworth response
        float cosw0 = std::cos(w0);
        float a0 = 1.0f + alpha;

        return FilterCoeff{
            { (1.0f + cosw0) / (2.0f * a0),
                -(1.0f + cosw0) / a0,
                (1.0f + cosw0) / (2.0f * a0) },
            { 1.0f,
                -2.0f * cosw0 / a0,
                (1.0f - alpha) / a0 },
            sampleFrequency
        };
    }

    FilterCoeff DesignBandPass(float centerFrequency, float bandwidth, float sampleFrequency)
    {
        float w0 = 2.0f * M_PI * centerFrequency / sampleFrequency;
        float q = centerFrequency / bandwidth;
        float alpha = std::sin(w0) * (2.0f * q);
        float cosw0 = std::cos(w0);
        float a0 = 1.0f + alpha;

        return FilterCoeff{
            { std::sin(w0) / (2.0f * a0),
                0.0f,
                -std::sin(w0) / (2.0f * a0) },
            { 1.0f,
                -2.0f * cosw0 / a0,
                (1.0f - alpha) / a0 },
            sampleFrequency
        };
    }

    template<std::size_t NumberOfPoints>
    class FilterFrequencyResponse
    {
    public:
        FilterFrequencyResponse(FilterCoeff filter, const std::string& filterType, const std::string& passType)
            : frequencyResponse(filter.b, filter.a, filter.sampleFrequency)
        {
            const auto [frequencies, magnitudes, phases] = frequencyResponse.Calculate();

            responsePlot.xlabel("Frequency (Hz)");
            responsePlot.ylabel("Magnitude (dB)");
            responsePlot.xtics().logscale();
            responsePlot.grid().show();
            responsePlot.legend().show();
            responsePlot.drawCurve(frequencies, magnitudes).label("Magnitude Response");

            phasePlot.xlabel("Frequency (Hz)");
            phasePlot.ylabel("Phase (degrees)");
            phasePlot.xtics().logscale();
            phasePlot.grid().show();
            phasePlot.legend().show();
            phasePlot.drawCurve(frequencies, phases).label("Phase Response");

            Figure figure = { { responsePlot, phasePlot } };
            figure.title(filterType + " " + passType + " Filter Frequency Response");
            figure.size(800, 600);
            figure.save("build/" + filterType + " " + passType + ".pdf");
        }

    private:
        analysis::FrequencyResponse<float, NumberOfPoints> frequencyResponse;
        Plot responsePlot, phasePlot;
    };
}

int main()
{
    const float sampleFrequency = 1000.0f;
    FilterFrequencyResponse<10000> iirLowPassFilter{ DesignLowPass(150.0f, sampleFrequency), "IIR", "Low pass" };
    FilterFrequencyResponse<10000> iirHighPassFilter{ DesignHighPass(100.0f, sampleFrequency), "IIR", "High pass" };
    FilterFrequencyResponse<10000> iirBandPassFilter{ DesignBandPass(250.0f, 100.0f, sampleFrequency), "IIR", "Band pass" };

    return 0;
}
