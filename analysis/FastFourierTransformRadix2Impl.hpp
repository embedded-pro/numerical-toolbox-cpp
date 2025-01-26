#ifndef ANALYSIS_FAST_FOURIER_TRANSFORM_RADIX_2_IMPL_HPP
#define ANALYSIS_FAST_FOURIER_TRANSFORM_RADIX_2_IMPL_HPP

#include "analysis/FastFourierTransform.hpp"
#include "infra/util/BoundedVector.hpp"
#include "math/ComplexNumber.hpp"

namespace analysis
{
    template<typename QNumberType, std::size_t Length>
    class FastFourierTransformRadix2Impl
        : public FastFourierTransform<QNumberType>
    {
        static_assert((Length & (Length - 1)) == 0, "FastFourierTransformRadix2Impl size must be a power of 2");

    public:
        using VectorComplex = typename FastFourierTransform<QNumberType>::VectorComplex;
        using VectorReal = typename FastFourierTransform<QNumberType>::VectorReal;

        explicit FastFourierTransformRadix2Impl(TwiddleFactors<QNumberType, Length / 2>& twinddleFactors);

        constexpr std::size_t NumberOfPoints() const override;

        VectorComplex& Forward(VectorReal& input) override;
        VectorReal& Inverse(VectorComplex& input) override;

    private:
        const std::size_t log2_n = FastFourierTransform<QNumberType>::Log2(Length);
        const std::size_t radix = 2;
        const std::size_t radixBits = FastFourierTransform<QNumberType>::Log2(radix);
        TwiddleFactors<QNumberType, Length / 2>& twinddleFactors;
        typename infra::BoundedVector<math::Complex<QNumberType>>::template WithMaxSize<Length> frequencyDomain;
        typename infra::BoundedVector<QNumberType>::template WithMaxSize<Length> timeDomain;
    };

    /// Implementation ///

    template<typename QNumberType, std::size_t Length>
    FastFourierTransformRadix2Impl<QNumberType, Length>::FastFourierTransformRadix2Impl(TwiddleFactors<QNumberType, Length / 2>& twinddleFactors)
        : twinddleFactors(twinddleFactors)
    {}

    template<typename QNumberType, std::size_t Length>
    constexpr std::size_t FastFourierTransformRadix2Impl<QNumberType, Length>::NumberOfPoints() const
    {
        return Length;
    }

    template<typename QNumberType, std::size_t Length>
    typename FastFourierTransformRadix2Impl<QNumberType, Length>::VectorComplex& FastFourierTransformRadix2Impl<QNumberType, Length>::Forward(FastFourierTransformRadix2Impl<QNumberType, Length>::VectorReal& input)
    {
        for (const auto& data : input)
            frequencyDomain.push_back(math::Complex<QNumberType>{ data, 0.0f });

        for (std::size_t i = 0; i < Length; ++i)
        {
            auto j = FastFourierTransform<QNumberType>::template BitReverse(i, log2_n, radixBits, radix);

            if (i < j)
                std::swap(frequencyDomain[i], frequencyDomain[j]);
        }

        for (auto step = 2; step <= Length; step *= 2)
        {
            auto halfStep = step / 2;
            auto k = 0;
            auto stepFactor = Length / step;

            for (size_t i = 0; i < Length; i += step)
            {
                k = 0;
                for (size_t j = i; j < i + halfStep; ++j)
                {
                    math::Complex<QNumberType>& a = frequencyDomain[j];
                    math::Complex<QNumberType>& b = frequencyDomain[j + halfStep];
                    math::Complex twiddle = twinddleFactors[k * stepFactor];

                    math::Complex temp = b * twiddle;
                    b = (a - temp);
                    a = (a + temp);

                    k++;
                }
            }
        }

        return frequencyDomain;
    }

    template<typename QNumberType, std::size_t Length>
    typename FastFourierTransformRadix2Impl<QNumberType, Length>::VectorReal& FastFourierTransformRadix2Impl<QNumberType, Length>::Inverse(FastFourierTransformRadix2Impl<QNumberType, Length>::VectorComplex& input)
    {
        frequencyDomain = input;

        for (auto& value : frequencyDomain)
            value = math::Complex<QNumberType>{ value.Real(), -value.Imaginary() };

        for (std::size_t i = 0; i < Length; ++i)
        {
            auto j = FastFourierTransform<QNumberType>::template BitReverse(i, log2_n, radixBits, radix);

            if (i < j)
                std::swap(frequencyDomain[i], frequencyDomain[j]);
        }

        for (auto step = radix; step <= Length; step *= radix)
        {
            auto halfStep = step / radix;
            auto k = 0;
            auto stepFactor = Length / step;

            for (size_t i = 0; i < Length; i += step)
            {
                k = 0;
                for (size_t j = i; j < i + halfStep; ++j)
                {
                    math::Complex<QNumberType>& a = frequencyDomain[j];
                    math::Complex<QNumberType>& b = frequencyDomain[j + halfStep];
                    math::Complex twiddle = twinddleFactors[k * stepFactor];

                    math::Complex temp = b * twiddle;
                    b = (a - temp);
                    a = (a + temp);

                    k++;
                }
            }
        }

        for (auto& value : frequencyDomain)
            value = math::Complex<QNumberType>{ value.Real(), -value.Imaginary() };

        timeDomain.clear();
        for (const auto& value : frequencyDomain)
            timeDomain.push_back(QNumberType(value.Real() / static_cast<float>(Length)));

        return timeDomain;
    }
}

#endif
