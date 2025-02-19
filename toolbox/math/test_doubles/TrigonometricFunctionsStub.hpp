#ifndef MATH_TRIGONOMETRIC_FUNCTIONS_STUB_HPP
#define MATH_TRIGONOMETRIC_FUNCTIONS_STUB_HPP

#include "toolbox/math/TrigonometricFunctions.hpp"
#include <algorithm>
#include <cmath>

namespace math
{
    template<typename QNumberType>
    class TrigonometricFunctionsStub
        : public TrigonometricFunctions<QNumberType>
    {
    public:
        constexpr static float _2_PI = 6.2831853f;
        constexpr static float MAX = 0.99999f;

        QNumberType Cosine(const QNumberType& angle) const override
        {
            really_assert(angle >= math::Lowest<QNumberType>() && angle <= math::Max<QNumberType>());
            return QNumberType(std::clamp(std::cos((math::ToFloat(angle) / MAX) * _2_PI), math::Lowest<QNumberType>(), math::Max<QNumberType>()));
        }

        QNumberType Sine(const QNumberType& angle) const override
        {
            really_assert(angle >= math::Lowest<QNumberType>() && angle <= math::Max<QNumberType>());
            return QNumberType(std::clamp(std::sin((math::ToFloat(angle) / MAX) * _2_PI), math::Lowest<QNumberType>(), math::Max<QNumberType>()));
        }

        QNumberType Arctangent(const QNumberType& angle) const override
        {
            really_assert(angle >= math::Lowest<QNumberType>() && angle <= math::Max<QNumberType>());
            return QNumberType(std::clamp(std::atan((math::ToFloat(angle) / MAX) * _2_PI), math::Lowest<QNumberType>(), math::Max<QNumberType>()));
        }

        QNumberType Phase(const QNumberType& real, const QNumberType& imag) const override
        {
            really_assert(real >= math::Lowest<QNumberType>() && real <= math::Max<QNumberType>());
            really_assert(imag >= math::Lowest<QNumberType>() && imag <= math::Max<QNumberType>());

            float atan2 = std::atan2((math::ToFloat(imag) / MAX) * _2_PI, (math::ToFloat(real) / MAX) * _2_PI);

            return QNumberType(std::clamp(atan2, math::Lowest<QNumberType>(), math::Max<QNumberType>()));
        }
    };
}

#endif
