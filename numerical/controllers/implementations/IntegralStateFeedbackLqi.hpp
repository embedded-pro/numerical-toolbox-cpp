#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/controllers/implementations/Lqr.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/LinearTimeInvariant.hpp"
#include "numerical/math/Matrix.hpp"

namespace controllers
{
    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>
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

        OPTIMIZE_FOR_SPEED InputVector ComputeControl(
            const StateVector& x,
            const OutputVector& reference,
            const OutputVector& measured);

        void Reset();

        [[nodiscard]] const GainStateMatrix& GetGainState() const;
        [[nodiscard]] const GainIntegralMatrix& GetGainIntegral() const;

    private:
        GainStateMatrix gainState{};
        GainIntegralMatrix gainIntegral{};
        IntegralVector integral{};
        T sampleTime{};
    };

    // Implementation //

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>
    IntegralStateFeedbackLqi<T, StateSize, InputSize, OutputSize>::IntegralStateFeedbackLqi(
        const GainStateMatrix& kx, const GainIntegralMatrix& ki, T ts)
        : gainState{ kx }
        , gainIntegral{ ki }
        , sampleTime{ ts }
    {}

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>
    IntegralStateFeedbackLqi<T, StateSize, InputSize, OutputSize>::IntegralStateFeedbackLqi(
        const math::LinearTimeInvariant<T, StateSize, InputSize, OutputSize>& plant,
        const math::SquareMatrix<T, AugmentedSize>& Q,
        const math::SquareMatrix<T, InputSize>& R,
        T ts)
        : sampleTime{ ts }
    {
        math::SquareMatrix<T, AugmentedSize> Aa{};
        math::Matrix<T, AugmentedSize, InputSize> Ba{};

        Aa.SetBlock(plant.A, 0, 0);
        Aa.SetBlock(plant.C * T(-ts), StateSize, 0);
        for (std::size_t r = 0; r < OutputSize; ++r)
            Aa.at(StateSize + r, StateSize + r) = T(1);
        Ba.SetBlock(plant.B, 0, 0);

        Lqr<T, AugmentedSize, InputSize> lqr{ Aa, Ba, Q, R };
        const auto& Ka = lqr.GetGain();

        gainState = Ka.template GetBlock<InputSize, StateSize>(0, 0);
        gainIntegral = Ka.template GetBlock<InputSize, OutputSize>(0, StateSize);
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>
    OPTIMIZE_FOR_SPEED
        typename IntegralStateFeedbackLqi<T, StateSize, InputSize, OutputSize>::InputVector
        IntegralStateFeedbackLqi<T, StateSize, InputSize, OutputSize>::ComputeControl(
            const StateVector& x,
            const OutputVector& reference,
            const OutputVector& measured)
    {
        auto error = reference - measured;
        integral = integral + error * sampleTime;
        return (gainState * x + gainIntegral * integral) * T(-1);
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>
    void IntegralStateFeedbackLqi<T, StateSize, InputSize, OutputSize>::Reset()
    {
        integral = IntegralVector{};
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>
    const typename IntegralStateFeedbackLqi<T, StateSize, InputSize, OutputSize>::GainStateMatrix&
    IntegralStateFeedbackLqi<T, StateSize, InputSize, OutputSize>::GetGainState() const
    {
        return gainState;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>
    const typename IntegralStateFeedbackLqi<T, StateSize, InputSize, OutputSize>::GainIntegralMatrix&
    IntegralStateFeedbackLqi<T, StateSize, InputSize, OutputSize>::GetGainIntegral() const
    {
        return gainIntegral;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class IntegralStateFeedbackLqi<float, 2, 1, 1>;
#endif
}
