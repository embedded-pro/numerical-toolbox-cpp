#ifndef ANALYSIS_DISCRETE_COSINE_TRANSFORM_HPP
#define ANALYSIS_DISCRETE_COSINE_TRANSFORM_HPP

#include "analysis/FastFourierTransform.hpp"
#include "infra/util/BoundedVector.hpp"
#include "math/ComplexNumber.hpp"

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
    };

    // Implementation //

    template<typename QNumberType, std::size_t Length>
    DiscreteConsineTransform<QNumberType, Length>::DiscreteConsineTransform(FastFourierTransform<QNumberType>& fft)
        : fft(fft)
    {}

    template<typename QNumberType, std::size_t Length>
    typename DiscreteConsineTransform<QNumberType, Length>::VectorReal& DiscreteConsineTransform<QNumberType, Length>::Forward(VectorReal& input)
    {
        return output;
    }

    template<typename QNumberType, std::size_t Length>
    typename DiscreteConsineTransform<QNumberType, Length>::VectorReal& DiscreteConsineTransform<QNumberType, Length>::Inverse(VectorReal& input)
    {
        return output;
    }
}

#endif
