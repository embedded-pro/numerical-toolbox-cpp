#pragma once

#include <cmath>
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/solvers/GaussianElimination.hpp"

namespace solvers
{
    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t MaxIterations = 100>
    class DiscreteAlgebraicRiccatiEquation
    {
        static_assert(math::detail::is_supported_type_v<T>,
            "DiscreteAlgebraicRiccatiEquation only supports float or QNumber types");
        static_assert(math::detail::is_valid_dimensions_v<StateSize, StateSize>,
            "State dimensions must be positive");
        static_assert(math::detail::is_valid_dimensions_v<InputSize, InputSize>,
            "Input dimensions must be positive");

    public:
        using StateMatrix = math::SquareMatrix<T, StateSize>;
        using InputMatrix = math::Matrix<T, StateSize, InputSize>;
        using InputWeightMatrix = math::SquareMatrix<T, InputSize>;

        DiscreteAlgebraicRiccatiEquation() = default;

        StateMatrix Solve(const StateMatrix& A, const InputMatrix& B, const StateMatrix& Q, const InputWeightMatrix& R) const;

    private:
        bool HasConverged(const StateMatrix& P, const StateMatrix& Pprev, float tolerance) const;
    };

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t MaxIterations>
    OPTIMIZE_FOR_SPEED
        typename DiscreteAlgebraicRiccatiEquation<T, StateSize, InputSize, MaxIterations>::StateMatrix
        DiscreteAlgebraicRiccatiEquation<T, StateSize, InputSize, MaxIterations>::Solve(
            const StateMatrix& A, const InputMatrix& B, const StateMatrix& Q, const InputWeightMatrix& R) const
    {
        StateMatrix P = Q;

        for (std::size_t iter = 0; iter < MaxIterations; ++iter)
        {
            StateMatrix Pprev = P;

            auto BtP = B.Transpose() * P;
            auto S = R + BtP * B;

            auto BtPA = BtP * A;
            auto K_part = SolveSystem<T, InputSize, StateSize>(S, BtPA);

            auto AtP = A.Transpose() * P;
            auto AtPA = AtP * A;
            auto AtPB = AtP * B;
            P = AtPA - AtPB * K_part + Q;

            if (HasConverged(P, Pprev, 1e-9f))
                break;
        }

        return P;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t MaxIterations>
    bool DiscreteAlgebraicRiccatiEquation<T, StateSize, InputSize, MaxIterations>::HasConverged(
        const StateMatrix& P, const StateMatrix& Pprev, float tolerance) const
    {
        for (std::size_t i = 0; i < StateSize; ++i)
        {
            for (std::size_t j = 0; j < StateSize; ++j)
            {
                float diff;
                if constexpr (std::is_same_v<T, float>)
                    diff = std::abs(P.at(i, j) - Pprev.at(i, j));
                else
                    diff = std::abs(P.at(i, j).ToFloat() - Pprev.at(i, j).ToFloat());

                if (diff > tolerance)
                    return false;
            }
        }
        return true;
    }
}
