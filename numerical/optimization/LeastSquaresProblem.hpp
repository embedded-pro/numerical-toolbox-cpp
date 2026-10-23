#pragma once

#include "numerical/math/Matrix.hpp"
#include <cstddef>
#include <type_traits>

namespace optimization
{
    template<typename T, typename State, std::size_t Residuals, std::size_t Parameters>
    class LeastSquaresProblem
    {
        static_assert(std::is_floating_point_v<T>, "LeastSquaresProblem supports floating-point types only");

    public:
        virtual ~LeastSquaresProblem() = default;

        [[nodiscard]] virtual math::Vector<T, Residuals> Residual(const State& x) const = 0;
        [[nodiscard]] virtual math::Matrix<T, Residuals, Parameters> Jacobian(const State& x) const = 0;
        [[nodiscard]] virtual State Retract(const State& x, const math::Vector<T, Parameters>& delta) const = 0;
    };
}
