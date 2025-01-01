#ifndef MATH_HYPERBOLIC_FUNCTIONS_HPP

#include "math/QNumber.hpp"

namespace math
{
    template<typename QNumberType>
    class HyperbolicFunctions
    {
    public:
        virtual QNumberType Cosine(const QNumberType& value) const = 0;
        virtual QNumberType Sine(const QNumberType& value) const = 0;
        virtual QNumberType Arctangent(const QNumberType& value) const = 0;
    };
}

#endif
