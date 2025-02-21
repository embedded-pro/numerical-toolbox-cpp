#include "numerical/analysis/FrequencyResponse.hpp"
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

    class Window
    {
    public:
        virtual float Calculate(std::size_t n, std::size_t order) = 0;
    };

    class HammingWindow
        : public Window
    {
    public:
        float Calculate(std::size_t n, std::size_t order) override
        {
            return 0.54f - 0.46f * std::cos(2.0f * M_PI * n / order);
        }
    };

    class HanningWindow
        : public Window
    {
    public:
        float Calculate(std::size_t n, std::size_t order) override
        {
            return 0.5f * (1.0f - std::cos(2.0f * M_PI * n / order));
        }
    };

    class BlackmanWindow
        : public Window
    {
    public:
        float Calculate(std::size_t n, std::size_t order) override
        {
            return 0.42f - 0.5f * std::cos(2.0f * M_PI * n / order) + 0.08f * std::cos(4.0f * M_PI * n / order);
        }
    };

    class RectangularWindow
        : public Window
    {
    public:
        float Calculate(std::size_t, std::size_t) override
        {
            return 1.0f;
        }
    };

    FilterCoeff DesignFirLowPass(float cutofFrequency, float sampleFrequency, std::size_t order, Window& window)
    {
        std::vector<float> h(order + 1);
        auto wc = 2.0f * M_PI * cutofFrequency / sampleFrequency;
        auto half_order = order / 2;

        for (auto n = 0; n <= order; n++)
        {
            if (n == half_order)
                h[n] = wc / M_PI;
            else
                h[n] = std::sin(wc * (n - half_order)) / (M_PI * (n - half_order));

            h[n] *= window.Calculate(n, order);
        }

        return { h, {}, sampleFrequency };
    }

    FilterCoeff DesignFirHighPass(float cutofFrequency, float sampleFrequency, std::size_t order, Window& window)
    {
        std::vector<float> h(order + 1);
        auto wc = 2.0f * M_PI * cutofFrequency / sampleFrequency;
        auto half_order = order / 2;

        for (auto n = 0; n <= order; n++)
        {
            if (n == half_order)
                h[n] = 1.0 - wc / M_PI;
            else
                h[n] = -std::sin(wc * (n - half_order)) / (M_PI * (n - half_order));

            h[n] *= window.Calculate(n, order);
        }

        return { h, {}, sampleFrequency };
    }

    FilterCoeff DesignFirBandPass(float lowerFrequency, float upperFrequency, float sampleFrequency, std::size_t order, Window& window)
    {
        std::vector<float> h(order + 1);
        auto wc1 = 2.0f * M_PI * lowerFrequency / sampleFrequency;
        auto wc2 = 2.0f * M_PI * upperFrequency / sampleFrequency;
        auto half_order = order / 2;

        for (auto n = 0; n <= order; n++)
        {
            if (n == half_order)
                h[n] = (wc2 - wc1) / M_PI;
            else
                h[n] = (std::sin(wc2 * (n - half_order)) - std::sin(wc1 * (n - half_order))) / (M_PI * (n - half_order));

            h[n] *= window.Calculate(n, order);
        }

        return { h, {}, sampleFrequency };
    }

    FilterCoeff DesignIirLowPass(float cutofFrequency, float sampleFrequency)
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

    FilterCoeff DesignIirHighPass(float cutofFrequency, float sampleFrequency)
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

    FilterCoeff DesignIirBandPass(float centerFrequency, float bandwidth, float sampleFrequency)
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
    const std::size_t order = 50;
    HanningWindow hanning;
    HammingWindow hamming;
    BlackmanWindow blackman;
    RectangularWindow rectangular;

    FilterFrequencyResponse<10000> iirLowPassFilter{ DesignIirLowPass(150.0f, sampleFrequency), "IIR", "Low pass" };
    FilterFrequencyResponse<10000> iirHighPassFilter{ DesignIirHighPass(100.0f, sampleFrequency), "IIR", "High pass" };
    FilterFrequencyResponse<10000> iirBandPassFilter{ DesignIirBandPass(250.0f, 100.0f, sampleFrequency), "IIR", "Band pass" };

    FilterFrequencyResponse<10000> hanningFirLowPassFilter{ DesignFirLowPass(150.0f, sampleFrequency, order, hanning), "FIR with Hanning window", "Low pass" };
    FilterFrequencyResponse<10000> hanningFirHighPassFilter{ DesignFirHighPass(100.0f, sampleFrequency, order, hanning), "FIR with Hanning window", "High pass" };
    FilterFrequencyResponse<10000> hanningFirBandPassFilter{ DesignFirBandPass(100.0f, 200.0f, sampleFrequency, order, hanning), "FIR with Hanning window", "Band pass" };

    FilterFrequencyResponse<10000> hammingFirLowPassFilter{ DesignFirLowPass(150.0f, sampleFrequency, order, hamming), "FIR with Hamming window", "Low pass" };
    FilterFrequencyResponse<10000> hammingFirHighPassFilter{ DesignFirHighPass(100.0f, sampleFrequency, order, hamming), "FIR with Hamming window", "High pass" };
    FilterFrequencyResponse<10000> hammingFirBandPassFilter{ DesignFirBandPass(100.0f, 200.0f, sampleFrequency, order, hamming), "FIR with Hamming window", "Band pass" };

    FilterFrequencyResponse<10000> blackmanFirLowPassFilter{ DesignFirLowPass(150.0f, sampleFrequency, order, blackman), "FIR with Blackman window", "Low pass" };
    FilterFrequencyResponse<10000> blackmanFirHighPassFilter{ DesignFirHighPass(100.0f, sampleFrequency, order, blackman), "FIR with Blackman window", "High pass" };
    FilterFrequencyResponse<10000> blackmanFirBandPassFilter{ DesignFirBandPass(100.0f, 200.0f, sampleFrequency, order, blackman), "FIR with Blackman window", "Band pass" };

    FilterFrequencyResponse<10000> rectangularFirLowPassFilter{ DesignFirLowPass(150.0f, sampleFrequency, order, rectangular), "FIR with Rectangular window", "Low pass" };
    FilterFrequencyResponse<10000> rectangularFirHighPassFilter{ DesignFirHighPass(100.0f, sampleFrequency, order, rectangular), "FIR with Rectangular window", "High pass" };
    FilterFrequencyResponse<10000> rectangularFirBandPassFilter{ DesignFirBandPass(100.0f, 200.0f, sampleFrequency, order, rectangular), "FIR with Rectangular window", "Band pass" };

    return 0;
}
