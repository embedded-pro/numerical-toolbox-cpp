#ifndef MATH_TRIGONOMETRIC_FUNCTIONS_HPP
#define MATH_TRIGONOMETRIC_FUNCTIONS_HPP

#include "math/QNumber.hpp"

namespace math
{
    template<typename QNumberType>
    class TrigonometricFunctions
    {
    public:
        virtual QNumberType Cosine(const QNumberType& angle) const = 0;
        virtual QNumberType Sine(const QNumberType& angle) const = 0;
        virtual QNumberType Arctangent(const QNumberType& value) const = 0;
        virtual QNumberType Phase(const QNumberType& real, const QNumberType& imag) const = 0;
    };
}

#endif
