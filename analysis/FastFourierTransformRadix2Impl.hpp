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

        VectorComplex& Forward(VectorReal& input) override;
        VectorReal& Inverse(VectorComplex& input) override;

    private:
        void ResetFrequencyDomain();
        void ResetTimeDomain();
        void BitReversePermutation();
        void Calculate();

    private:
        const std::size_t log2_n = FastFourierTransform<QNumberType>::Log2(Length);
        const std::size_t radix = 2;
        const std::size_t radixBits = FastFourierTransform<QNumberType>::Log2(radix);
        TwiddleFactors<QNumberType, Length / 2>& twinddleFactors;
        typename infra::BoundedVector<math::Complex<QNumberType>>::template WithMaxSize<Length> frequencyDomain;
        typename infra::BoundedVector<QNumberType>::template WithMaxSize<Length> timeDomain;
    };

    template<typename QNumberType, std::size_t Length>
    FastFourierTransformRadix2Impl<QNumberType, Length>::FastFourierTransformRadix2Impl(TwiddleFactors<QNumberType, Length / 2>& twinddleFactors)
        : twinddleFactors(twinddleFactors)
    {}

    template<typename QNumberType, std::size_t Length>
    void FastFourierTransformRadix2Impl<QNumberType, Length>::BitReversePermutation()
    {
        for (std::size_t i = 0; i < Length; ++i)
        {
            auto j = FastFourierTransform<QNumberType>::template BitReverse(i, log2_n, radixBits, radix);
            if (i < j)
                std::swap(frequencyDomain[i], frequencyDomain[j]);
        }
    }

    template<typename QNumberType, std::size_t Length>
    void FastFourierTransformRadix2Impl<QNumberType, Length>::Calculate()
    {
        for (auto step = 2; step <= Length; step *= 2)
        {
            auto halfStep = step / 2;
            auto stepFactor = Length / step;

            for (size_t i = 0; i < Length; i += step)
            {
                auto k = 0;
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
    }

    template<typename QNumberType, std::size_t Length>
    typename FastFourierTransformRadix2Impl<QNumberType, Length>::VectorComplex& FastFourierTransformRadix2Impl<QNumberType, Length>::Forward(FastFourierTransformRadix2Impl<QNumberType, Length>::VectorReal& input)
    {
        ResetFrequencyDomain();

        for (std::size_t i = 0; i < input.size(); i++)
            frequencyDomain[i] = (math::Complex<QNumberType>{ input[i], 0.0f });

        BitReversePermutation();
        Calculate();

        return frequencyDomain;
    }

    template<typename QNumberType, std::size_t Length>
    typename FastFourierTransformRadix2Impl<QNumberType, Length>::VectorReal& FastFourierTransformRadix2Impl<QNumberType, Length>::Inverse(FastFourierTransformRadix2Impl<QNumberType, Length>::VectorComplex& input)
    {
        frequencyDomain = input;

        for (auto& value : frequencyDomain)
            value = math::Complex<QNumberType>{ value.Real(), -value.Imaginary() };

        BitReversePermutation();
        Calculate();

        for (auto& value : frequencyDomain)
            value = math::Complex<QNumberType>{ value.Real(), -value.Imaginary() };

        ResetTimeDomain();

        for (std::size_t i = 0; i < frequencyDomain.size(); i++)
            timeDomain[i] = (QNumberType(frequencyDomain[i].Real() / static_cast<float>(Length)));

        return timeDomain;
    }

    template<typename QNumberType, std::size_t Length>
    void FastFourierTransformRadix2Impl<QNumberType, Length>::ResetFrequencyDomain()
    {
        frequencyDomain.clear();
        frequencyDomain.resize(frequencyDomain.max_size());
    }

    template<typename QNumberType, std::size_t Length>
    void FastFourierTransformRadix2Impl<QNumberType, Length>::ResetTimeDomain()
    {
        timeDomain.clear();
        timeDomain.resize(timeDomain.max_size());
    }
}

#endif
