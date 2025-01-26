#ifndef ANALYSIS_POWER_SPECTRAL_DENSITY_HPP
#define ANALYSIS_POWER_SPECTRAL_DENSITY_HPP

#include "analysis/FastFourierTransform.hpp"
#include "infra/util/ReallyAssert.hpp"
#include "math/QNumber.hpp"
#include "windowing/Windowing.hpp"
#include <utility>

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

        std::pair<VectorReal&, VectorReal&> Calculate(const VectorReal& input)
        {
            really_assert(input.size() >= SegmentSize);
            auto scale = segmentSizeInverted * samplingTimeInSeconds;

            y.clear();
            x.clear();
            y.resize(SegmentSize / 2 + 1);
            x.resize(SegmentSize / 2 + 1);

            for (std::size_t i = 0, segmentCount = 0; i + SegmentSize <= input.size(); i += step, ++segmentCount)
            {
                segment.clear();
                segment.resize(SegmentSize);
                for (std::size_t j = 0; j < SegmentSize; ++j)
                    segment[j] = (QNumberType(input[i + j] * window(j, SegmentSize)));

                spectrum = fft.Forward(segment);

                for (std::size_t k = 0; k <= SegmentSize / 2; ++k)
                    y[k] += QNumberType(math::ToFloat(MagnitudeSquared(k)) / static_cast<float>(SegmentSize));
            }

            for (std::size_t i = 0; i < y.size(); ++i)
            {
                y[i] *= frequencyResolution;
                x[i] = QNumberType(math::ToFloat(frequencyResolution) * static_cast<float>(i));
            }

            return std::make_pair(std::ref(x), std::ref(y));
        }

    private:
        QNumberType MagnitudeSquared(std::size_t k)
        {
            return spectrum[k].Real() * spectrum[k].Real() + spectrum[k].Imaginary() * spectrum[k].Imaginary();
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
        typename FastFourierTransform<QNumberType>::VectorComplex::template WithMaxSize<SegmentSize> spectrum;
        typename FastFourierTransform<QNumberType>::VectorReal::template WithMaxSize<SegmentSize / 2 + 1> x;
        typename FastFourierTransform<QNumberType>::VectorReal::template WithMaxSize<SegmentSize / 2 + 1> y;
    };
}

#endif
