#ifndef ANALYSIS_DISCRETE_COSINE_TRANSFORM_HPP
#define ANALYSIS_DISCRETE_COSINE_TRANSFORM_HPP

#include "infra/util/BoundedVector.hpp"
#include "toolbox/analysis/FastFourierTransform.hpp"
#include "toolbox/math/ComplexNumber.hpp"
#include <cmath>

namespace analysis
{
    template<typename QNumberType, std::size_t Length>
    class DiscreteConsineTransform
    {
        static_assert((Length & (Length - 1)) == 0, "DiscreteConsineTransform size must be a power of 2");
        static_assert(math::is_qnumber<QNumberType>::value || std::is_floating_point<QNumberType>::value,
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
        typename VectorComplex::template WithMaxSize<Length> complexBuffer;
        static constexpr float PI = 3.14159265358979323846f;
    };

    // Implementation //

    template<typename QNumberType, std::size_t Length>
    DiscreteConsineTransform<QNumberType, Length>::DiscreteConsineTransform(FastFourierTransform<QNumberType>& fft)
        : fft(fft)
    {
        output.resize(Length);
        complexBuffer.resize(Length);
    }

    template<typename QNumberType, std::size_t Length>
    typename DiscreteConsineTransform<QNumberType, Length>::VectorReal& DiscreteConsineTransform<QNumberType, Length>::Forward(VectorReal& input)
    {
        auto& fftResult = fft.Forward(input);

        output[0] = QNumberType(math::ToFloat(fftResult[0].Real()) / std::sqrt(static_cast<float>(Length)));

        for (std::size_t k = 1; k < Length; ++k)
        {
            float angle = -k * PI / (2.0f * Length);
            float scale = 2.0f / std::sqrt(static_cast<float>(Length));

            float real = math::ToFloat(fftResult[k].Real());
            float imag = math::ToFloat(fftResult[k].Imaginary());

            output[k] = QNumberType((real * std::cos(angle) - imag * std::sin(angle)) * scale);
        }

        return output;
    }

    template<typename QNumberType, std::size_t Length>
    typename DiscreteConsineTransform<QNumberType, Length>::VectorReal& DiscreteConsineTransform<QNumberType, Length>::Inverse(VectorReal& input)
    {
        complexBuffer[0] = math::Complex<QNumberType>{ QNumberType(math::ToFloat(input[0]) * std::sqrt(static_cast<float>(Length))), QNumberType(0.0f) };

        for (std::size_t k = 1; k < Length; ++k)
        {
            float angle = k * PI / (2.0f * Length);
            float scale = std::sqrt(static_cast<float>(Length)) / 2.0f;

            float value = math::ToFloat(input[k]) * scale;
            complexBuffer[k] = math::Complex<QNumberType>{ QNumberType(value * std::cos(angle)), QNumberType(value * std::sin(angle)) };
        }

        return fft.Inverse(complexBuffer);
    }
}

#endif
