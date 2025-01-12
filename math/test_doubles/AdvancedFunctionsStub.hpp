#ifndef MATH_ADVANCED_FUNCTIONS_STUB_HPP
#define MATH_ADVANCED_FUNCTIONS_STUB_HPP

#include "math/AdvancedFunctions.hpp"

namespace math
{
    template<typename QNumberType>
    class AdvancedFunctionsStub
        : public AdvancedFunctions<QNumberType>
    {
    public:
        QNumberType Modulus(const QNumberType& real, const QNumberType& imag) const override
        {
            really_assert(real > -1.0f && real < 1.0f);
            really_assert(imag > -1.0f && imag < 1.0f);
            return QNumberType(std::sqrt(std::pow(math::ToFloat(real), 2.0f) + std::pow(math::ToFloat(imag), 2.0f)));
        }

        QNumberType NaturalLogarithm(const QNumberType& value) const override
        {
            really_assert(value > -1.0f && value < 1.0f);
            return QNumberType(std::log(math::ToFloat(value)));
        }

        QNumberType SquareRoot(const QNumberType& value) const override
        {
            really_assert(value >= 0.0f && value < 1.0f);
            return QNumberType(std::sqrt(math::ToFloat(value)));
        }
    };
}

#endif
