#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "infra/util/ReallyAssert.hpp"
#include "numerical/controllers/interfaces/StateFeedbackController.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/LinearTimeInvariant.hpp"
#include "numerical/solvers/SingularValueDecomposition.hpp"
#include <type_traits>

namespace controllers
{
    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t Steps = 1>
    class DeadbeatControl
        : public StateFeedbackController<T, StateSize, InputSize>
    {
        static_assert(std::is_floating_point_v<T>, "DeadbeatControl supports floating-point types");
        static_assert(Steps > 0, "Steps must be positive");
        static_assert(StateSize <= Steps * InputSize,
            "StateSize must be <= Steps * InputSize for full row rank reachability matrix");

        using Base = StateFeedbackController<T, StateSize, InputSize>;
        static constexpr std::size_t ReachSize = Steps * InputSize;

    public:
        using typename Base::InputVector;
        using typename Base::StateVector;
        using StateMatrix = math::SquareMatrix<T, StateSize>;
        using InputMatrix = math::Matrix<T, StateSize, InputSize>;
        using GainMatrix = math::Matrix<T, InputSize, StateSize>;

        struct PrecomputedGains
        {};

        DeadbeatControl(const StateMatrix& A, const InputMatrix& B);
        DeadbeatControl(const math::LinearTimeInvariant<T, StateSize, InputSize>& plant);
        DeadbeatControl(PrecomputedGains, const GainMatrix& stateGain, const GainMatrix& refGain);

        OPTIMIZE_FOR_SPEED InputVector ComputeControl(const StateVector& state) override;

        void SetReference(const StateVector& reference);
        void ClearReference();

        [[nodiscard]] const GainMatrix& GetStateGain() const;
        [[nodiscard]] const GainMatrix& GetReferenceGain() const;

    private:
        void ComputeGains(const StateMatrix& A, const InputMatrix& B);

        GainMatrix gainState{};
        GainMatrix gainRef{};
        StateVector reference{};
    };

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t Steps>
    DeadbeatControl<T, StateSize, InputSize, Steps>::DeadbeatControl(
        const StateMatrix& A, const InputMatrix& B)
    {
        ComputeGains(A, B);
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t Steps>
    DeadbeatControl<T, StateSize, InputSize, Steps>::DeadbeatControl(
        const math::LinearTimeInvariant<T, StateSize, InputSize>& plant)
        : DeadbeatControl(plant.A, plant.B)
    {}

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t Steps>
    DeadbeatControl<T, StateSize, InputSize, Steps>::DeadbeatControl(
        PrecomputedGains, const GainMatrix& stateGain, const GainMatrix& refGain)
        : gainState{ stateGain }
        , gainRef{ refGain }
    {}

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t Steps>
    OPTIMIZE_FOR_SPEED
        typename DeadbeatControl<T, StateSize, InputSize, Steps>::InputVector
        DeadbeatControl<T, StateSize, InputSize, Steps>::ComputeControl(const StateVector& state)
    {
        return gainRef * reference - gainState * state;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t Steps>
    void DeadbeatControl<T, StateSize, InputSize, Steps>::SetReference(const StateVector& ref)
    {
        reference = ref;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t Steps>
    void DeadbeatControl<T, StateSize, InputSize, Steps>::ClearReference()
    {
        reference = StateVector{};
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t Steps>
    const typename DeadbeatControl<T, StateSize, InputSize, Steps>::GainMatrix&
    DeadbeatControl<T, StateSize, InputSize, Steps>::GetStateGain() const
    {
        return gainState;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t Steps>
    const typename DeadbeatControl<T, StateSize, InputSize, Steps>::GainMatrix&
    DeadbeatControl<T, StateSize, InputSize, Steps>::GetReferenceGain() const
    {
        return gainRef;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t Steps>
    void DeadbeatControl<T, StateSize, InputSize, Steps>::ComputeGains(
        const StateMatrix& A, const InputMatrix& B)
    {
        math::Matrix<T, StateSize, ReachSize> gamma{};
        InputMatrix Apow = B;
        for (std::size_t i = 0; i < Steps; ++i)
        {
            gamma.SetBlock(Apow, 0, (Steps - 1 - i) * InputSize);
            Apow = A * Apow;
        }

        StateMatrix AN = StateMatrix::Identity();
        for (std::size_t i = 0; i < Steps; ++i)
            AN = A * AN;

        static constexpr T kSvdTol = T(1e-5f);
        solvers::SingularValueDecomposition<T, StateSize, ReachSize> svd;
        svd.Decompose(gamma);
        really_assert(svd.Rank(kSvdTol) == StateSize);

        auto gammaPinv = svd.PseudoInverse(kSvdTol);

        gainRef = gammaPinv.template GetBlock<InputSize, StateSize>(0, 0);
        gainState = gainRef * AN;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class DeadbeatControl<float, 1, 1, 1>;
    extern template class DeadbeatControl<float, 2, 1, 2>;
    extern template class DeadbeatControl<float, 2, 2, 1>;
#endif
}
