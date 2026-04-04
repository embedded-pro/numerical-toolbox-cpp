#pragma once

#include "numerical/math/Matrix.hpp"

namespace regularization
{
    template<typename QNumberType, std::size_t Size>
    class Regularization
    {
    public:
        using Vector = math::Vector<QNumberType, Size>;
        virtual QNumberType Calculate(const Vector& parameters) const = 0;
        virtual Vector Gradient(const Vector& parameters) const = 0;
    };
}
