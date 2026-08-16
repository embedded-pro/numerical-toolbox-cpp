#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/controllers/implementations/Lqr.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/LinearTimeInvariant.hpp"
#include "numerical/math/Matrix.hpp"
#include <optional>

namespace controllers
{
    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize, std::size_t MaxIterations = 300>
    class IntegralStateFeedbackLqi
    {
        static_assert(std::is_floating_point_v<T>, "IntegralStateFeedbackLqi supports floating-point types");

        static constexpr std::size_t AugmentedSize = StateSize + OutputSize;

    public:
        using StateVector = math::Vector<T, StateSize>;
        using InputVector = math::Vector<T, InputSize>;
        using OutputVector = math::Vector<T, OutputSize>;
        using GainStateMatrix = math::Matrix<T, InputSize, StateSize>;
        using GainIntegralMatrix = math::Matrix<T, InputSize, OutputSize>;
        using IntegralVector = math::Vector<T, OutputSize>;

        IntegralStateFeedbackLqi(const GainStateMatrix& kx, const GainIntegralMatrix& ki, T ts);

        IntegralStateFeedbackLqi(
            const math::LinearTimeInvariant<T, StateSize, InputSize, OutputSize>& plant,
            const math::SquareMatrix<T, AugmentedSize>& Q,
            const math::SquareMatrix<T, InputSize>& R,
            T ts);

        [[nodiscard]] static std::optional<IntegralStateFeedbackLqi> TryCreate(
            const math::LinearTimeInvariant<T, StateSize, InputSize, OutputSize>& plant,
            const math::SquareMatrix<T, AugmentedSize>& Q,
            const math::SquareMatrix<T, InputSize>& R,
            T ts);

        OPTIMIZE_FOR_SPEED InputVector ComputeControl(
            const StateVector& x,
            const OutputVector& reference,
            const OutputVector& measured);

        void Reset();

        [[nodiscard]] const GainStateMatrix& GetGainState() const;
        [[nodiscard]] const GainIntegralMatrix& GetGainIntegral() const;

    private:
        using AugmentedStateMatrix = math::SquareMatrix<T, AugmentedSize>;
        using AugmentedInputMatrix = math::Matrix<T, AugmentedSize, InputSize>;

        static void BuildAugmentedSystem(
            const math::LinearTimeInvariant<T, StateSize, InputSize, OutputSize>& plant,
            T ts,
            AugmentedStateMatrix& Aa,
            AugmentedInputMatrix& Ba);

        GainStateMatrix gainState{};
        GainIntegralMatrix gainIntegral{};
        IntegralVector integral{};
        T sampleTime{};
    };

    // Implementation //

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize, std::size_t MaxIterations>
    IntegralStateFeedbackLqi<T, StateSize, InputSize, OutputSize, MaxIterations>::IntegralStateFeedbackLqi(
        const GainStateMatrix& kx, const GainIntegralMatrix& ki, T ts)
        : gainState{ kx }
        , gainIntegral{ ki }
        , sampleTime{ ts }
    {}

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize, std::size_t MaxIterations>
    IntegralStateFeedbackLqi<T, StateSize, InputSize, OutputSize, MaxIterations>::IntegralStateFeedbackLqi(
        const math::LinearTimeInvariant<T, StateSize, InputSize, OutputSize>& plant,
        const math::SquareMatrix<T, AugmentedSize>& Q,
        const math::SquareMatrix<T, InputSize>& R,
        T ts)
        : sampleTime{ ts }
    {
        AugmentedStateMatrix Aa{};
        AugmentedInputMatrix Ba{};
        BuildAugmentedSystem(plant, ts, Aa, Ba);

        Lqr<T, AugmentedSize, InputSize, MaxIterations> lqr{ Aa, Ba, Q, R };
        const auto& Ka = lqr.GetGain();

        gainState = Ka.template GetBlock<InputSize, StateSize>(0, 0);
        gainIntegral = Ka.template GetBlock<InputSize, OutputSize>(0, StateSize);
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize, std::size_t MaxIterations>
    std::optional<IntegralStateFeedbackLqi<T, StateSize, InputSize, OutputSize, MaxIterations>>
    IntegralStateFeedbackLqi<T, StateSize, InputSize, OutputSize, MaxIterations>::TryCreate(
        const math::LinearTimeInvariant<T, StateSize, InputSize, OutputSize>& plant,
        const math::SquareMatrix<T, AugmentedSize>& Q,
        const math::SquareMatrix<T, InputSize>& R,
        T ts)
    {
        AugmentedStateMatrix Aa{};
        AugmentedInputMatrix Ba{};
        BuildAugmentedSystem(plant, ts, Aa, Ba);

        auto lqr = Lqr<T, AugmentedSize, InputSize, MaxIterations>::TryCreate(Aa, Ba, Q, R);
        if (!lqr)
            return std::nullopt;

        const auto& Ka = lqr->GetGain();
        return IntegralStateFeedbackLqi(
            Ka.template GetBlock<InputSize, StateSize>(0, 0),
            Ka.template GetBlock<InputSize, OutputSize>(0, StateSize),
            ts);
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize, std::size_t MaxIterations>
    void IntegralStateFeedbackLqi<T, StateSize, InputSize, OutputSize, MaxIterations>::BuildAugmentedSystem(
        const math::LinearTimeInvariant<T, StateSize, InputSize, OutputSize>& plant,
        T ts,
        AugmentedStateMatrix& Aa,
        AugmentedInputMatrix& Ba)
    {
        Aa.SetBlock(plant.A, 0, 0);
        Aa.SetBlock(plant.C * T(-ts), StateSize, 0);
        for (std::size_t r = 0; r < OutputSize; ++r)
            Aa.at(StateSize + r, StateSize + r) = T(1);
        Ba.SetBlock(plant.B, 0, 0);
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize, std::size_t MaxIterations>
    OPTIMIZE_FOR_SPEED
        typename IntegralStateFeedbackLqi<T, StateSize, InputSize, OutputSize, MaxIterations>::InputVector
        IntegralStateFeedbackLqi<T, StateSize, InputSize, OutputSize, MaxIterations>::ComputeControl(
            const StateVector& x,
            const OutputVector& reference,
            const OutputVector& measured)
    {
        auto error = reference - measured;
        integral = integral + error * sampleTime;
        return (gainState * x + gainIntegral * integral) * T(-1);
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize, std::size_t MaxIterations>
    void IntegralStateFeedbackLqi<T, StateSize, InputSize, OutputSize, MaxIterations>::Reset()
    {
        integral = IntegralVector{};
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize, std::size_t MaxIterations>
    const typename IntegralStateFeedbackLqi<T, StateSize, InputSize, OutputSize, MaxIterations>::GainStateMatrix&
    IntegralStateFeedbackLqi<T, StateSize, InputSize, OutputSize, MaxIterations>::GetGainState() const
    {
        return gainState;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize, std::size_t MaxIterations>
    const typename IntegralStateFeedbackLqi<T, StateSize, InputSize, OutputSize, MaxIterations>::GainIntegralMatrix&
    IntegralStateFeedbackLqi<T, StateSize, InputSize, OutputSize, MaxIterations>::GetGainIntegral() const
    {
        return gainIntegral;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class IntegralStateFeedbackLqi<float, 2, 1, 1>;
    extern template class IntegralStateFeedbackLqi<float, 2, 1, 1, 1>;
#endif
}
