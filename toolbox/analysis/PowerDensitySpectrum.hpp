#ifndef ANALYSIS_POWER_SPECTRAL_DENSITY_HPP
#define ANALYSIS_POWER_SPECTRAL_DENSITY_HPP

#include "infra/util/ReallyAssert.hpp"
#include "toolbox/analysis/FastFourierTransform.hpp"
#include "toolbox/math/ComplexNumber.hpp"
#include "toolbox/math/QNumber.hpp"
#include "toolbox/windowing/Windowing.hpp"

namespace analysis
{
    template<typename QNumberType, std::size_t SegmentSize, typename Fft, typename TwindleFactor, std::size_t Overlap>
    class PowerSpectralDensity
    {
        static_assert(math::is_qnumber<QNumberType>::value ||
                          std::is_floating_point<QNumberType>::value,
            "PowerSpectralDensity can only be instantiated with math::QNumber types.");

        static_assert(std::is_base_of<FastFourierTransform<QNumberType>, Fft>::value,
            "Fft has to be derived from FastFourierTransform.");

        static_assert((SegmentSize % 2) == 0,
            "SegmentSize has to be multiple of 2.");

        static_assert(Overlap == 0 || Overlap == 25 || Overlap == 50 || Overlap == 75,
            "Only values supported by overlap are: 0%, 25%, 50% and 75%.");

    public:
        explicit PowerSpectralDensity(windowing::Window<QNumberType>& window, QNumberType samplingTimeInSeconds)
            : window(window)
            , samplingTimeInSeconds(samplingTimeInSeconds)
            , frequencyResolution(QNumberType(samplingTimeInSeconds * segmentSizeInverted))
        {}

        using VectorReal = typename FastFourierTransform<QNumberType>::VectorReal;
        using VectorComplex = typename FastFourierTransform<QNumberType>::VectorComplex;

        VectorReal& Calculate(const VectorReal& input)
        {
            really_assert(input.size() >= SegmentSize);

            ResetOutput();

            for (std::size_t i = 0, segmentCount = 0; i + SegmentSize <= input.size(); i += step, ++segmentCount)
            {
                ResetSegment();

                for (std::size_t j = 0; j < SegmentSize; ++j)
                    segment[j] = (QNumberType(input[i + j] * window(j, SegmentSize)));

                auto& spectrum = fft.Forward(segment);

                for (std::size_t k = 0; k <= SegmentSize / 2; ++k)
                    y[k] += QNumberType(math::ToFloat(MagnitudeSquared(spectrum[k])) / static_cast<float>(SegmentSize));
            }

            for (std::size_t i = 0; i < y.size(); ++i)
                y[i] *= frequencyResolution;

            return y;
        }

    private:
        QNumberType MagnitudeSquared(math::Complex<QNumberType>& data)
        {
            return data.Real() * data.Real() + data.Imaginary() * data.Imaginary();
        }

        void ResetOutput()
        {
            y.clear();
            y.resize(SegmentSize / 2 + 1);
        }

        void ResetSegment()
        {
            segment.clear();
            segment.resize(SegmentSize);
        }

    private:
        static constexpr QNumberType segmentSizeInverted = QNumberType(1.0f / static_cast<float>(SegmentSize));
        static constexpr std::size_t overlapSize = (SegmentSize * Overlap) / 100;
        static constexpr std::size_t step = SegmentSize - overlapSize;
        QNumberType frequencyResolution;
        QNumberType scale;
        windowing::Window<QNumberType>& window;
        QNumberType samplingTimeInSeconds;
        TwindleFactor twindleFacors;
        Fft fft{ twindleFacors };
        typename infra::BoundedVector<QNumberType>::template WithMaxSize<SegmentSize> segment;
        typename FastFourierTransform<QNumberType>::VectorReal::template WithMaxSize<SegmentSize / 2 + 1> y;
    };
}

#endif
