#pragma once

#include "numerical/controllers/interfaces/LqrController.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/solvers/DiscreteAlgebraicRiccatiEquation.hpp"
#include "numerical/solvers/GaussianElimination.hpp"

namespace controllers
{
    template<typename T, std::size_t StateSize, std::size_t InputSize>
    class Lqr
        : public LqrController<T, StateSize, InputSize>
    {
    public:
        using typename LqrController<T, StateSize, InputSize>::StateMatrix;
        using typename LqrController<T, StateSize, InputSize>::InputMatrix;
        using typename LqrController<T, StateSize, InputSize>::StateVector;
        using typename LqrController<T, StateSize, InputSize>::InputVector;
        using typename LqrController<T, StateSize, InputSize>::GainMatrix;
        using InputWeightMatrix = math::SquareMatrix<T, InputSize>;

        Lqr(const StateMatrix& A, const InputMatrix& B, const StateMatrix& Q, const InputWeightMatrix& R);
        explicit Lqr(const GainMatrix& precomputedGain);

        InputVector ComputeControl(const StateVector& state) override;
        [[nodiscard]] const GainMatrix& GetGain() const override;
        [[nodiscard]] const StateMatrix& GetRiccatiSolution() const;

    private:
        void ComputeGain(const StateMatrix& A, const InputMatrix& B, const StateMatrix& P, const InputWeightMatrix& R);

        GainMatrix gain;
        StateMatrix riccatiSolution;
    };

    template<typename T, std::size_t StateSize, std::size_t InputSize>
    Lqr<T, StateSize, InputSize>::Lqr(
        const StateMatrix& A, const InputMatrix& B, const StateMatrix& Q, const InputWeightMatrix& R)
    {
        solvers::DiscreteAlgebraicRiccatiEquation<T, StateSize, InputSize> dare;
        riccatiSolution = dare.Solve(A, B, Q, R);
        ComputeGain(A, B, riccatiSolution, R);
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize>
    Lqr<T, StateSize, InputSize>::Lqr(const GainMatrix& precomputedGain)
        : gain(precomputedGain)
    {}

    template<typename T, std::size_t StateSize, std::size_t InputSize>
    OPTIMIZE_FOR_SPEED
        typename Lqr<T, StateSize, InputSize>::InputVector
        Lqr<T, StateSize, InputSize>::ComputeControl(const StateVector& state)
    {
        return gain * state * T(-1.0f);
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize>
    const typename Lqr<T, StateSize, InputSize>::GainMatrix&
    Lqr<T, StateSize, InputSize>::GetGain() const
    {
        return gain;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize>
    const typename Lqr<T, StateSize, InputSize>::StateMatrix&
    Lqr<T, StateSize, InputSize>::GetRiccatiSolution() const
    {
        return riccatiSolution;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize>
    OPTIMIZE_FOR_SPEED void Lqr<T, StateSize, InputSize>::ComputeGain(
        const StateMatrix& A, const InputMatrix& B, const StateMatrix& P, const InputWeightMatrix& R)
    {
        auto BtP = B.Transpose() * P;
        auto S = R + BtP * B;
        auto BtPA = BtP * A;
        gain = solvers::SolveSystem<T, InputSize, StateSize>(S, BtPA);
    }

    extern template class Lqr<float, 1, 1>;
    extern template class Lqr<float, 2, 1>;
    extern template class Lqr<float, 3, 1>;
    extern template class Lqr<float, 2, 2>;

    extern template class Lqr<math::Q15, 1, 1>;
    extern template class Lqr<math::Q15, 2, 1>;

    extern template class Lqr<math::Q31, 1, 1>;
    extern template class Lqr<math::Q31, 2, 1>;
}
