#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/dynamics/EulerLagrangeDynamics.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/solvers/GaussianElimination.hpp"

namespace dynamics
{
    template<typename T, std::size_t Dof>
    class EulerLagrangeSolver
    {
        static_assert(std::is_floating_point_v<T>,
            "EulerLagrangeSolver only supports floating-point types");
        static_assert(math::detail::is_valid_dimensions_v<Dof, Dof>,
            "Degrees of freedom must be positive");

    public:
        using StateVector = math::Vector<T, Dof>;
        using MassMatrix = math::SquareMatrix<T, Dof>;

        EulerLagrangeSolver() = default;

        // Forward dynamics: computes qDDot = M(q)^{-1} * (tau - C(q, qDot) - g(q))
        OPTIMIZE_FOR_SPEED StateVector ForwardDynamics(const EulerLagrangeDynamics<T, Dof>& model,
            const StateVector& q, const StateVector& qDot, const StateVector& tau) const;

        // Inverse dynamics: computes tau = M(q) * qDDot + C(q, qDot) + g(q)
        OPTIMIZE_FOR_SPEED StateVector InverseDynamics(const EulerLagrangeDynamics<T, Dof>& model,
            const StateVector& q, const StateVector& qDot, const StateVector& qDDot) const;
    };

    template<typename T, std::size_t Dof>
    OPTIMIZE_FOR_SPEED
        typename EulerLagrangeSolver<T, Dof>::StateVector
        EulerLagrangeSolver<T, Dof>::ForwardDynamics(const EulerLagrangeDynamics<T, Dof>& model,
            const StateVector& q, const StateVector& qDot, const StateVector& tau) const
    {
        auto M = model.ComputeMassMatrix(q);
        auto C = model.ComputeCoriolisTerms(q, qDot);
        auto g = model.ComputeGravityTerms(q);

        auto rhs = tau - C - g;

        solvers::GaussianElimination<T, Dof> solver;
        return solver.Solve(M, rhs);
    }

    template<typename T, std::size_t Dof>
    OPTIMIZE_FOR_SPEED
        typename EulerLagrangeSolver<T, Dof>::StateVector
        EulerLagrangeSolver<T, Dof>::InverseDynamics(const EulerLagrangeDynamics<T, Dof>& model,
            const StateVector& q, const StateVector& qDot, const StateVector& qDDot) const
    {
        auto M = model.ComputeMassMatrix(q);
        auto C = model.ComputeCoriolisTerms(q, qDot);
        auto g = model.ComputeGravityTerms(q);

        return M * qDDot + C + g;
    }

    extern template class EulerLagrangeSolver<float, 1>;
    extern template class EulerLagrangeSolver<float, 2>;
    extern template class EulerLagrangeSolver<float, 3>;
}
