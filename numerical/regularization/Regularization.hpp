#pragma once

#include "numerical/math/Matrix.hpp"
#include "numerical/math/QNumber.hpp"

namespace regularization
{
    template<typename T, std::size_t Size>
    class Regularization
    {
        static_assert(std::is_floating_point_v<T> || math::is_qnumber_v<T>, "Regularization supports floating-point and QNumber types");

    public:
        using Vector = math::Vector<T, Size>;
        virtual T Calculate(const Vector& parameters) const = 0;
        virtual Vector Gradient(const Vector& parameters) const = 0;
        virtual ~Regularization() = default;
    };
}
