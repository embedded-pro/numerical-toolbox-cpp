#ifndef MATH_HYPERBOLIC_FUNCTIONS_STUB_HPP
#define MATH_HYPERBOLIC_FUNCTIONS_STUB_HPP

#include "math/HyperbolicFunctions.hpp"

namespace dsp
{
    template<typename QNumberType>
    class HyperbolicFunctionsStub
        : public HyperbolicFunctions<QNumberType>
    {
    public:
        virtual QNumberType Cosine(const QNumberType& value) const = 0;
        virtual QNumberType Sine(const QNumberType& value) const = 0;
        virtual QNumberType Arctangent(const QNumberType& value) const = 0;
    };
}

#endif
