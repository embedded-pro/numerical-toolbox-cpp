#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/QNumber.hpp"
#include "numerical/math/Tolerance.hpp"
#include "numerical/solvers/GaussianElimination.hpp"
#include <cmath>

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
        StateMatrix Iterate(const StateMatrix& P, const StateMatrix& A, const InputMatrix& B,
            const StateMatrix& Q, const InputWeightMatrix& R) const;
        bool HasConverged(const StateMatrix& P, const StateMatrix& Pprev, float tolerance) const;
    };

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t MaxIterations>
    OPTIMIZE_FOR_SPEED
        typename DiscreteAlgebraicRiccatiEquation<T, StateSize, InputSize, MaxIterations>::StateMatrix
        DiscreteAlgebraicRiccatiEquation<T, StateSize, InputSize, MaxIterations>::Iterate(
            const StateMatrix& P, const StateMatrix& A, const InputMatrix& B,
            const StateMatrix& Q, const InputWeightMatrix& R) const
    {
        auto BtP = B.Transpose() * P;
        auto S = R + BtP * B;
        auto K_part = SolveSystem<T, InputSize, StateSize>(S, BtP * A);

        auto AtP = A.Transpose() * P;
        return AtP * A - AtP * B * K_part + Q;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t MaxIterations>
    typename DiscreteAlgebraicRiccatiEquation<T, StateSize, InputSize, MaxIterations>::StateMatrix
    DiscreteAlgebraicRiccatiEquation<T, StateSize, InputSize, MaxIterations>::Solve(
        const StateMatrix& A, const InputMatrix& B, const StateMatrix& Q, const InputWeightMatrix& R) const
    {
        StateMatrix P = Q;

        for (std::size_t iter = 0; iter < MaxIterations; ++iter)
        {
            StateMatrix Pprev = P;
            P = Iterate(P, A, B, Q, R);

            if (HasConverged(P, Pprev, math::Tolerance<T>()))
                break;
        }

        return P;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t MaxIterations>
    bool DiscreteAlgebraicRiccatiEquation<T, StateSize, InputSize, MaxIterations>::HasConverged(
        const StateMatrix& P, const StateMatrix& Pprev, float tolerance) const
    {
        for (std::size_t i = 0; i < StateSize; ++i)
            for (std::size_t j = 0; j < StateSize; ++j)
                if (std::abs(math::ToFloat(P.at(i, j)) - math::ToFloat(Pprev.at(i, j))) > tolerance)
                    return false;

        return true;
    }

    extern template class DiscreteAlgebraicRiccatiEquation<float, 1, 1>;
    extern template class DiscreteAlgebraicRiccatiEquation<float, 2, 1>;
    extern template class DiscreteAlgebraicRiccatiEquation<float, 4, 1>;
}
