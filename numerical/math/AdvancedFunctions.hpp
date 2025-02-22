#ifndef MATH_ADVANCED_FUNCTIONS_HPP
#define MATH_ADVANCED_FUNCTIONS_HPP

#include "numerical/math/QNumber.hpp"

namespace math
{
    template<typename QNumberType>
    class AdvancedFunctions
    {
    public:
        virtual QNumberType Modulus(const QNumberType& real, const QNumberType& imag) const = 0;
        virtual QNumberType NaturalLogarithm(const QNumberType& value) const = 0;
        virtual QNumberType SquareRoot(const QNumberType& value) const = 0;
    };
}

#endif
