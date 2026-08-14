// Copyright (c) 2024 Numerical Toolbox Contributors
// SPDX-License-Identifier: MIT

#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include <cstddef>
#include <type_traits>

namespace robust_control
{
    template<typename T, std::size_t Order>
    class ActiveDisturbanceRejectionControl
    {
        static_assert(std::is_floating_point_v<T>, "ActiveDisturbanceRejectionControl supports floating-point types");
        static_assert(Order > 0, "ActiveDisturbanceRejectionControl requires Order > 0");

    public:
        using StateVector = math::Vector<T, Order + 1>;
        using ControlVector = math::Vector<T, Order>;

        ActiveDisturbanceRejectionControl(T observerBandwidth, T controlBandwidth, T b0, T sampleTime);

        OPTIMIZE_FOR_SPEED T Compute(T reference, T measuredOutput, T actualApplied);
        OPTIMIZE_FOR_SPEED T Compute(T reference, T measuredOutput);
        void Reset();

        [[nodiscard]] static StateVector ObserverGainFromBandwidth(T wo);
        [[nodiscard]] static ControlVector ControlGainFromBandwidth(T wc);

        [[nodiscard]] const StateVector& EstimatedState() const;
        [[nodiscard]] T AppliedPrev() const;

    private:
        StateVector xhat{};
        StateVector observerGain{};
        ControlVector controlGain{};
        T b0;
        T sampleTime;
        T appliedPrev{ T{ 0 } };
    };

    namespace detail
    {
        constexpr std::size_t BinomialCoeff(std::size_t n, std::size_t k)
        {
            if (k == 0 || k == n)
                return 1;
            if (k > n)
                return 0;
            std::size_t result{ 1 };
            for (std::size_t i = 0; i < k; ++i)
            {
                result *= (n - i);
                result /= (i + 1);
            }
            return result;
        }
    }

    template<typename T, std::size_t Order>
    ActiveDisturbanceRejectionControl<T, Order>::ActiveDisturbanceRejectionControl(
        T observerBandwidth, T controlBandwidth, T b0, T sampleTime)
        : observerGain{ ObserverGainFromBandwidth(observerBandwidth) }
        , controlGain{ ControlGainFromBandwidth(controlBandwidth) }
        , b0{ b0 }
        , sampleTime{ sampleTime }
    {
        really_assert(observerBandwidth * sampleTime < T{ 0.5 } / static_cast<T>(Order));
    }

    template<typename T, std::size_t Order>
    OPTIMIZE_FOR_SPEED T ActiveDisturbanceRejectionControl<T, Order>::Compute(T reference, T measuredOutput, T actualApplied)
    {
        const T e = measuredOutput - xhat.at(0, 0);

        for (std::size_t i = 0; i <= Order; ++i)
            xhat.at(i, 0) += sampleTime * observerGain.at(i, 0) * e;

        for (std::size_t i = 0; i < Order; ++i)
            xhat.at(i, 0) += sampleTime * xhat.at(i + 1, 0);

        xhat.at(Order - 1, 0) += sampleTime * b0 * actualApplied;

        T u0 = controlGain.at(0, 0) * (reference - xhat.at(0, 0));
        for (std::size_t i = 1; i < Order; ++i)
            u0 -= controlGain.at(i, 0) * xhat.at(i, 0);

        const T u = (u0 - xhat.at(Order, 0)) / b0;
        appliedPrev = u;
        return u;
    }

    template<typename T, std::size_t Order>
    OPTIMIZE_FOR_SPEED T ActiveDisturbanceRejectionControl<T, Order>::Compute(T reference, T measuredOutput)
    {
        return Compute(reference, measuredOutput, appliedPrev);
    }

    template<typename T, std::size_t Order>
    void ActiveDisturbanceRejectionControl<T, Order>::Reset()
    {
        xhat = StateVector{};
        appliedPrev = T{ 0 };
    }

    template<typename T, std::size_t Order>
    typename ActiveDisturbanceRejectionControl<T, Order>::StateVector
    ActiveDisturbanceRejectionControl<T, Order>::ObserverGainFromBandwidth(T wo)
    {
        StateVector gains{};
        const std::size_t n = Order + 1;
        T woPow{ wo };
        for (std::size_t i = 0; i < n; ++i)
        {
            const auto coeff = static_cast<T>(detail::BinomialCoeff(n, i + 1));
            gains.at(i, 0) = coeff * woPow;
            woPow *= wo;
        }
        return gains;
    }

    template<typename T, std::size_t Order>
    typename ActiveDisturbanceRejectionControl<T, Order>::ControlVector
    ActiveDisturbanceRejectionControl<T, Order>::ControlGainFromBandwidth(T wc)
    {
        ControlVector gains{};
        T wcPow{ wc };
        for (std::size_t i = 0; i < Order; ++i)
        {
            const auto coeff = static_cast<T>(detail::BinomialCoeff(Order, i + 1));
            gains.at(Order - 1 - i, 0) = coeff * wcPow;
            wcPow *= wc;
        }
        return gains;
    }

    template<typename T, std::size_t Order>
    const typename ActiveDisturbanceRejectionControl<T, Order>::StateVector&
    ActiveDisturbanceRejectionControl<T, Order>::EstimatedState() const
    {
        return xhat;
    }

    template<typename T, std::size_t Order>
    T ActiveDisturbanceRejectionControl<T, Order>::AppliedPrev() const
    {
        return appliedPrev;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class ActiveDisturbanceRejectionControl<float, 2>;
#endif
}
