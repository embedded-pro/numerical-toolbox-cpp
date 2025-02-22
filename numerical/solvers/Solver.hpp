#ifndef SOLVERS_SOLVER_HPP
#define SOLVERS_SOLVER_HPP

#include "numerical/math/Matrix.hpp"

namespace solvers
{
    template<typename T, std::size_t N>
    class Solver
    {
        static_assert(math::detail::is_supported_type_v<T>,
            "Solver only supports float or QNumber types");
        static_assert(math::detail::is_valid_dimensions_v<N, N>,
            "Solver dimensions must be positive");

    public:
        using SolutionVector = math::Vector<T, N>;
        using InputMatrix = math::Matrix<T, N, N>;
        using InputVector = math::Vector<T, N>;

        virtual SolutionVector Solve(const InputMatrix& a, const InputVector& b) = 0;
    };
}

#endif
