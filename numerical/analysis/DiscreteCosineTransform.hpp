#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "infra/util/BoundedVector.hpp"
#include "numerical/analysis/FastFourierTransform.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/ComplexNumber.hpp"
#include "numerical/math/Math.hpp"
#include <numbers>

namespace analysis
{
    template<typename QNumberType, std::size_t Length>
    class DiscreteConsineTransform
    {
        static_assert((Length & (Length - 1)) == 0, "DiscreteConsineTransform size must be a power of 2");
        static_assert(math::is_qnumber<QNumberType>::value || std::is_floating_point_v<QNumberType>,
            "DiscreteConsineTransform can only be instantiated with math::QNumber types or floating point.");

    public:
        using VectorReal = typename FastFourierTransform<QNumberType>::VectorReal;
        using VectorComplex = typename FastFourierTransform<QNumberType>::VectorComplex;

        explicit DiscreteConsineTransform(FastFourierTransform<QNumberType>& fft);
        VectorReal& Forward(VectorReal& input);
        VectorReal& Inverse(VectorReal& input);

    private:
        FastFourierTransform<QNumberType>& fft;
        typename infra::BoundedVector<QNumberType>::template WithMaxSize<Length> output;
        typename infra::BoundedVector<QNumberType>::template WithMaxSize<Length> reordered;
        typename VectorComplex::template WithMaxSize<Length> complexBuffer;
    };

    // Implementation //

    template<typename QNumberType, std::size_t Length>
    DiscreteConsineTransform<QNumberType, Length>::DiscreteConsineTransform(FastFourierTransform<QNumberType>& fft)
        : fft(fft)
    {
        output.resize(Length);
        reordered.resize(Length);
        complexBuffer.resize(Length);
    }

    template<typename QNumberType, std::size_t Length>
    OPTIMIZE_FOR_SPEED
        typename DiscreteConsineTransform<QNumberType, Length>::VectorReal&
        DiscreteConsineTransform<QNumberType, Length>::Forward(VectorReal& input)
    {
        for (std::size_t n = 0; n < Length / 2; ++n)
        {
            reordered[n] = input[2 * n];
            reordered[Length - 1 - n] = input[2 * n + 1];
        }

        auto& fftResult = fft.Forward(reordered);

        output[0] = QNumberType(math::ToFloat(fftResult[0].Real()) / math::Sqrt(static_cast<float>(Length)));

        for (std::size_t k = 1; k < Length; ++k)
        {
            float angle = -static_cast<float>(k) * std::numbers::pi_v<float> / (2.0f * Length);
            float scale = 2.0f / math::Sqrt(static_cast<float>(Length));

            float real = math::ToFloat(fftResult[k].Real());
            float imag = math::ToFloat(fftResult[k].Imaginary());

            output[k] = QNumberType((real * math::Cos(angle) - imag * math::Sin(angle)) * scale);
        }

        return output;
    }

    template<typename QNumberType, std::size_t Length>
    typename DiscreteConsineTransform<QNumberType, Length>::VectorReal& DiscreteConsineTransform<QNumberType, Length>::Inverse(VectorReal& input)
    {
        float sqrtN = math::Sqrt(static_cast<float>(Length));

        complexBuffer[0] = math::Complex<QNumberType>{ QNumberType(math::ToFloat(input[0]) * sqrtN), QNumberType(0.0f) };

        for (std::size_t k = 1; k < Length; ++k)
        {
            float real = math::ToFloat(input[k]) * sqrtN / 2.0f;
            float imag = -math::ToFloat(input[Length - k]) * sqrtN / 2.0f;

            float angle = static_cast<float>(k) * std::numbers::pi_v<float> / (2.0f * static_cast<float>(Length));
            float cosine = math::Cos(angle);
            float sine = math::Sin(angle);

            complexBuffer[k] = math::Complex<QNumberType>{ QNumberType(real * cosine - imag * sine), QNumberType(real * sine + imag * cosine) };
        }

        auto& timeDomain = fft.Inverse(complexBuffer);

        for (std::size_t n = 0; n < Length / 2; ++n)
        {
            output[2 * n] = timeDomain[n];
            output[2 * n + 1] = timeDomain[Length - 1 - n];
        }

        return output;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class DiscreteConsineTransform<float, 8>;
    extern template class DiscreteConsineTransform<math::Q15, 8>;
    extern template class DiscreteConsineTransform<math::Q31, 8>;
#endif
}
