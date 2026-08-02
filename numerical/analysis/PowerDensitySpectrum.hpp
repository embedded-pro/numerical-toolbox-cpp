#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/analysis/FastFourierTransform.hpp"
#include "numerical/analysis/windowing/Windowing.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/ComplexNumber.hpp"
#include "numerical/math/QNumber.hpp"

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
#include "numerical/analysis/test/PowerDensitySpectrumTestSupport.hpp"
#endif

namespace analysis
{
    template<typename QNumberType, std::size_t SegmentSize, typename Fft, typename TwiddleFactor, std::size_t Overlap>
    class PowerSpectralDensity
    {
        static_assert(math::is_qnumber_v<QNumberType> ||
                          std::is_floating_point_v<QNumberType>,
            "PowerSpectralDensity can only be instantiated with math::QNumber types.");

        static_assert(std::is_base_of_v<FastFourierTransform<QNumberType>, Fft>,
            "Fft has to be derived from FastFourierTransform.");

        static_assert((SegmentSize % 2) == 0,
            "SegmentSize has to be multiple of 2.");

        static_assert(Overlap == 0 || Overlap == 25 || Overlap == 50 || Overlap == 75,
            "Only values supported by overlap are: 0%, 25%, 50% and 75%.");

    public:
        using VectorReal = typename FastFourierTransform<QNumberType>::VectorReal;
        using VectorComplex = typename FastFourierTransform<QNumberType>::VectorComplex;

        explicit PowerSpectralDensity(windowing::Window<QNumberType>& window, QNumberType samplingTimeInSeconds);

        OPTIMIZE_FOR_SPEED VectorReal& Calculate(const VectorReal& input);

    private:
        QNumberType MagnitudeSquared(const math::Complex<QNumberType>& data) const;
        void ResetOutput();
        void ResetSegment();

    private:
        static constexpr std::size_t overlapSize = (SegmentSize * Overlap) / 100;
        static constexpr std::size_t step = SegmentSize - overlapSize;
        windowing::Window<QNumberType>& window;
        QNumberType samplingTimeInSeconds;
        TwiddleFactor twiddleFactors;
        Fft fft{ twiddleFactors };
        typename infra::BoundedVector<QNumberType>::template WithMaxSize<SegmentSize> segment;
        typename FastFourierTransform<QNumberType>::VectorReal::template WithMaxSize<SegmentSize / 2 + 1> y;
    };

    /// Implementation ///

    template<typename QNumberType, std::size_t SegmentSize, typename Fft, typename TwiddleFactor, std::size_t Overlap>
    PowerSpectralDensity<QNumberType, SegmentSize, Fft, TwiddleFactor, Overlap>::PowerSpectralDensity(
        windowing::Window<QNumberType>& window, QNumberType samplingTimeInSeconds)
        : window(window)
        , samplingTimeInSeconds(samplingTimeInSeconds)
    {}

    template<typename QNumberType, std::size_t SegmentSize, typename Fft, typename TwiddleFactor, std::size_t Overlap>
    OPTIMIZE_FOR_SPEED typename PowerSpectralDensity<QNumberType, SegmentSize, Fft, TwiddleFactor, Overlap>::VectorReal&
    PowerSpectralDensity<QNumberType, SegmentSize, Fft, TwiddleFactor, Overlap>::Calculate(const VectorReal& input)
    {
        really_assert(input.size() >= SegmentSize);

        ResetOutput();

        std::size_t segmentCount = 0;

        for (std::size_t i = 0; i + SegmentSize <= input.size(); i += step)
        {
            ResetSegment();

            for (std::size_t j = 0; j < SegmentSize; ++j)
                segment[j] = QNumberType(input[i + j] * window(j, SegmentSize));

            auto& spectrum = fft.Forward(segment);

            for (std::size_t k = 0; k <= SegmentSize / 2; ++k)
                y[k] += QNumberType(math::ToFloat(MagnitudeSquared(spectrum[k])) / static_cast<float>(SegmentSize));

            ++segmentCount;
        }

        float normalization = math::ToFloat(samplingTimeInSeconds) /
                              (math::ToFloat(window.Power(SegmentSize)) * static_cast<float>(segmentCount));

        for (std::size_t i = 0; i < y.size(); ++i)
            y[i] = QNumberType(math::ToFloat(y[i]) * normalization);

        return y;
    }

    template<typename QNumberType, std::size_t SegmentSize, typename Fft, typename TwiddleFactor, std::size_t Overlap>
    QNumberType PowerSpectralDensity<QNumberType, SegmentSize, Fft, TwiddleFactor, Overlap>::MagnitudeSquared(
        const math::Complex<QNumberType>& data) const
    {
        return data.Real() * data.Real() + data.Imaginary() * data.Imaginary();
    }

    template<typename QNumberType, std::size_t SegmentSize, typename Fft, typename TwiddleFactor, std::size_t Overlap>
    void PowerSpectralDensity<QNumberType, SegmentSize, Fft, TwiddleFactor, Overlap>::ResetOutput()
    {
        y.clear();
        y.resize(SegmentSize / 2 + 1);
    }

    template<typename QNumberType, std::size_t SegmentSize, typename Fft, typename TwiddleFactor, std::size_t Overlap>
    void PowerSpectralDensity<QNumberType, SegmentSize, Fft, TwiddleFactor, Overlap>::ResetSegment()
    {
        segment.clear();
        segment.resize(SegmentSize);
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class PowerSpectralDensity<float, 512, test::FftStub<float, 512>, test::TwiddleFactorsStub<float, 256>, 50>;
    extern template class PowerSpectralDensity<float, 512, test::FftStub<float, 512>, test::TwiddleFactorsStub<float, 256>, 0>;
#endif
}
