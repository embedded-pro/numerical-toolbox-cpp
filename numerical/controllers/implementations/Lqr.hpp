#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "infra/util/ReallyAssert.hpp"
#include "numerical/controllers/interfaces/StateFeedbackController.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/LinearTimeInvariant.hpp"
#include "numerical/solvers/DiscreteAlgebraicRiccatiEquation.hpp"
#include "numerical/solvers/GaussianElimination.hpp"
#include <optional>

namespace controllers
{
    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t MaxIterations = 300>
    class Lqr
        : public StateFeedbackController<T, StateSize, InputSize>
    {
        using Base = StateFeedbackController<T, StateSize, InputSize>;

    public:
        using typename Base::InputVector;
        using typename Base::StateVector;
        using StateMatrix = math::SquareMatrix<T, StateSize>;
        using InputMatrix = math::Matrix<T, StateSize, InputSize>;
        using GainMatrix = math::Matrix<T, InputSize, StateSize>;
        using InputWeightMatrix = math::SquareMatrix<T, InputSize>;

        Lqr(const StateMatrix& A, const InputMatrix& B, const StateMatrix& Q, const InputWeightMatrix& R);
        explicit Lqr(const GainMatrix& precomputedGain);

        Lqr(const math::LinearTimeInvariant<T, StateSize, InputSize>& plant,
            const StateMatrix& Q, const InputWeightMatrix& R);

        [[nodiscard]] static std::optional<Lqr> TryCreate(
            const StateMatrix& A, const InputMatrix& B, const StateMatrix& Q, const InputWeightMatrix& R);

        InputVector ComputeControl(const StateVector& state) override;
        [[nodiscard]] const GainMatrix& GetGain() const;
        [[nodiscard]] const StateMatrix& GetRiccatiSolution() const;

    private:
        struct PrecomputedRiccatiTag
        {
        };

        Lqr(const StateMatrix& A, const InputMatrix& B, const StateMatrix& P, const InputWeightMatrix& R, PrecomputedRiccatiTag);

        void ComputeGain(const StateMatrix& A, const InputMatrix& B, const StateMatrix& P, const InputWeightMatrix& R);

        GainMatrix gain;
        StateMatrix riccatiSolution;
        bool riccatiSolutionAvailable = false;
    };

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t MaxIterations>
    Lqr<T, StateSize, InputSize, MaxIterations>::Lqr(
        const StateMatrix& A, const InputMatrix& B, const StateMatrix& Q, const InputWeightMatrix& R)
        : riccatiSolution([&A, &B, &Q, &R]
              {
                  auto r = solvers::DiscreteAlgebraicRiccatiEquation<T, StateSize, InputSize, MaxIterations>{}.Solve(A, B, Q, R);
                  really_assert(r.converged);
                  return r.value;
              }())
        , riccatiSolutionAvailable(true)
    {
        ComputeGain(A, B, riccatiSolution, R);
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t MaxIterations>
    Lqr<T, StateSize, InputSize, MaxIterations>::Lqr(const GainMatrix& precomputedGain)
        : gain(precomputedGain)
    {}

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t MaxIterations>
    Lqr<T, StateSize, InputSize, MaxIterations>::Lqr(
        const StateMatrix& A, const InputMatrix& B, const StateMatrix& P, const InputWeightMatrix& R, PrecomputedRiccatiTag)
        : riccatiSolution(P)
        , riccatiSolutionAvailable(true)
    {
        ComputeGain(A, B, riccatiSolution, R);
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t MaxIterations>
    std::optional<Lqr<T, StateSize, InputSize, MaxIterations>>
    Lqr<T, StateSize, InputSize, MaxIterations>::TryCreate(
        const StateMatrix& A, const InputMatrix& B, const StateMatrix& Q, const InputWeightMatrix& R)
    {
        auto result = solvers::DiscreteAlgebraicRiccatiEquation<T, StateSize, InputSize, MaxIterations>{}.Solve(A, B, Q, R);
        if (!result.converged)
            return std::nullopt;

        return Lqr(A, B, result.value, R, PrecomputedRiccatiTag{});
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t MaxIterations>
    OPTIMIZE_FOR_SPEED
        typename Lqr<T, StateSize, InputSize, MaxIterations>::InputVector
        Lqr<T, StateSize, InputSize, MaxIterations>::ComputeControl(const StateVector& state)
    {
        return gain * state * T(-1.0f);
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t MaxIterations>
    const typename Lqr<T, StateSize, InputSize, MaxIterations>::GainMatrix&
    Lqr<T, StateSize, InputSize, MaxIterations>::GetGain() const
    {
        return gain;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t MaxIterations>
    const typename Lqr<T, StateSize, InputSize, MaxIterations>::StateMatrix&
    Lqr<T, StateSize, InputSize, MaxIterations>::GetRiccatiSolution() const
    {
        really_assert(riccatiSolutionAvailable);
        return riccatiSolution;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t MaxIterations>
    Lqr<T, StateSize, InputSize, MaxIterations>::Lqr(
        const math::LinearTimeInvariant<T, StateSize, InputSize>& plant,
        const StateMatrix& Q, const InputWeightMatrix& R)
        : Lqr(plant.A, plant.B, Q, R)
    {}

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t MaxIterations>
    OPTIMIZE_FOR_SPEED void Lqr<T, StateSize, InputSize, MaxIterations>::ComputeGain(
        const StateMatrix& A, const InputMatrix& B, const StateMatrix& P, const InputWeightMatrix& R)
    {
        auto BtP = B.Transpose() * P;
        auto S = R + BtP * B;
        auto BtPA = BtP * A;
        gain = solvers::SolveSystem<T, InputSize, StateSize>(S, BtPA);
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class Lqr<float, 1, 1>;
    extern template class Lqr<float, 2, 1>;
    extern template class Lqr<float, 2, 1, 1>;
    extern template class Lqr<float, 3, 1>;
    extern template class Lqr<float, 4, 1>;
    extern template class Lqr<float, 2, 2>;

    extern template class Lqr<math::Q15, 1, 1>;
    extern template class Lqr<math::Q15, 2, 1>;

    extern template class Lqr<math::Q31, 1, 1>;
    extern template class Lqr<math::Q31, 2, 1>;
#endif
}
