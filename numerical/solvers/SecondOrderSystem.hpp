#pragma once

#include "numerical/math/Matrix.hpp"
#include <cstddef>
#include <type_traits>

namespace solvers
{
    template<typename T, std::size_t Dimension, std::size_t InputSize>
    class SecondOrderSystem
    {
        static_assert(std::is_floating_point_v<T>, "SecondOrderSystem supports floating-point types only");

    public:
        [[nodiscard]] virtual math::Vector<T, Dimension> Acceleration(
            const math::Vector<T, Dimension>& q,
            const math::Vector<T, Dimension>& qDot,
            const math::Vector<T, InputSize>& u) = 0;

    protected:
        ~SecondOrderSystem() = default;
    };
}
