#pragma once

#include "numerical/math/Matrix.hpp"
#include <cstddef>
#include <type_traits>

namespace solvers
{
    template<typename T, std::size_t StateSize, std::size_t InputSize>
    class OdeSystem
    {
        static_assert(std::is_floating_point_v<T>, "OdeSystem supports floating-point types only");

    public:
        virtual ~OdeSystem() = default;
        [[nodiscard]] virtual math::Vector<T, StateSize> Derivative(
            const math::Vector<T, StateSize>& x,
            const math::Vector<T, InputSize>& u,
            T t) = 0;
    };

    template<typename T, std::size_t StateSize>
    class OdeSystem<T, StateSize, 0>
    {
        static_assert(std::is_floating_point_v<T>, "OdeSystem supports floating-point types only");

    public:
        virtual ~OdeSystem() = default;
        [[nodiscard]] virtual math::Vector<T, StateSize> Derivative(
            const math::Vector<T, StateSize>& x,
            T t) = 0;
    };
}
