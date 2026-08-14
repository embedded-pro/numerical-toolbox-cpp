#pragma once
#include "infra/util/BoundedVector.hpp"
#include "numerical/math/ComplexNumber.hpp"
#include "numerical/math/QNumber.hpp"

namespace analysis
{
    template<typename QNumberType, std::size_t Length>
    class TwiddleFactors
    {
    public:
        virtual ~TwiddleFactors() = default;
        virtual math::Complex<QNumberType>& operator[](std::size_t n) = 0;
    };

    template<typename QNumberType>
    class FastFourierTransform
    {
        static_assert(math::is_qnumber<QNumberType>::value ||
                          std::is_floating_point_v<QNumberType>,
            "FastFourierTransform can only be instantiated with math::QNumber types.");

    public:
        virtual ~FastFourierTransform() = default;

        static constexpr std::size_t Log2(std::size_t n)
        {
            std::size_t result = 0;
            while (n > 1)
            {
                n >>= 1;
                ++result;
            }
            return result;
        }

        using VectorComplex = infra::BoundedVector<math::Complex<QNumberType>>;
        using VectorReal = infra::BoundedVector<QNumberType>;

        virtual VectorComplex& Forward(VectorReal& input) = 0;
        virtual VectorReal& Inverse(VectorComplex& input) = 0;

    protected:
        template<typename T>
        inline T BitReverse(T x, T logr_n, T bits, T radix)
        {
            T n = 0;

            for (T i = 0; i < logr_n; ++i)
            {
                n = (n << bits) | (x & (radix - 1));
                x >>= bits;
            }

            return n;
        }
    };
}
