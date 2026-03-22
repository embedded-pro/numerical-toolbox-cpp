#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/Matrix.hpp"

namespace dynamics
{
    template<typename T, std::size_t Dof>
    class EulerLagrangeDynamics
    {
        static_assert(std::is_floating_point_v<T>,
            "EulerLagrangeDynamics only supports floating-point types");
        static_assert(math::detail::is_valid_dimensions_v<Dof, Dof>,
            "Degrees of freedom must be positive");

    public:
        using StateVector = math::Vector<T, Dof>;
        using MassMatrix = math::SquareMatrix<T, Dof>;

        virtual ~EulerLagrangeDynamics() = default;

        virtual MassMatrix ComputeMassMatrix(const StateVector& q) const = 0;
        virtual StateVector ComputeCoriolisTerms(const StateVector& q, const StateVector& qDot) const = 0;
        virtual StateVector ComputeGravityTerms(const StateVector& q) const = 0;
    };
}
