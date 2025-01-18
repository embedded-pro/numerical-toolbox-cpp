#ifndef ANALYSIS_FAST_FOURIER_TRANSFORM_RADIX_4_IMPL_HPP
#define ANALYSIS_FAST_FOURIER_TRANSFORM_RADIX_4_IMPL_HPP

#include "analysis/FastFourierTransform.hpp"
#include "infra/util/BoundedVector.hpp"

namespace analysis
{
    template<typename QNumberType, std::size_t Length>
    class FastFourierTransformRadix4Impl
        : public FastFourierTransform<QNumberType>
    {
    public:
        infra::BoundedVector<QNumberType>& Forward(infra::BoundedVector<QNumberType>& input) override;
        infra::BoundedVector<QNumberType>& Inverse(infra::BoundedVector<QNumberType>& input) override;

    private:
        typename infra::BoundedVector<QNumberType>::template WithMaxSize<Length> result;
    };

    /// Implementation ///

    template<typename QNumberType, std::size_t Length>
    infra::BoundedVector<QNumberType>& FastFourierTransformRadix4Impl<QNumberType, Length>::Forward(infra::BoundedVector<QNumberType>& input)
    {
        return result;
    }

    template<typename QNumberType, std::size_t Length>
    infra::BoundedVector<QNumberType>& FastFourierTransformRadix4Impl<QNumberType, Length>::Inverse(infra::BoundedVector<QNumberType>& input)
    {
        return result;
    }
}

#endif
