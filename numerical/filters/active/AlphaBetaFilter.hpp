#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include <array>
#include "numerical/math/Math.hpp"
#include <cstddef>
#include <type_traits>

namespace filters
{
    template<typename T, std::size_t Order>
    class AlphaBetaFilter
    {
        static_assert(std::is_floating_point_v<T>, "AlphaBetaFilter supports floating-point types");
        static_assert(Order == 2 || Order == 3, "AlphaBetaFilter Order must be 2 or 3");

    public:
        struct Gains
        {
            T alpha;
            T beta;
        };

        AlphaBetaFilter(T alpha, T beta, T Ts)
        requires(Order == 2);

        AlphaBetaFilter(T alpha, T beta, T gamma, T Ts)
        requires(Order == 3);

        OPTIMIZE_FOR_SPEED T Filter(T measuredPosition);

        std::array<T, Order> State() const;

        void Reset(T position = T{});

        static Gains GainsFromTrackingIndex(T lambda);

    private:
        T samplePeriod{};
        T gainAlpha{};
        T gainBeta{};
        T gainGamma{};
        T betaOverTs{};
        T twoGammaOverTs2{};
        std::array<T, Order> state{};
        bool initialized{ false };
    };

    // Implementation //

    template<typename T, std::size_t Order>
    AlphaBetaFilter<T, Order>::AlphaBetaFilter(T alpha, T beta, T Ts)
    requires(Order == 2)
        : samplePeriod{ Ts }
        , gainAlpha{ alpha }
        , gainBeta{ beta }
        , gainGamma{ T{} }
        , betaOverTs{ beta / Ts }
        , twoGammaOverTs2{ T{} }
    {}

    template<typename T, std::size_t Order>
    AlphaBetaFilter<T, Order>::AlphaBetaFilter(T alpha, T beta, T gamma, T Ts)
    requires(Order == 3)
        : samplePeriod{ Ts }
        , gainAlpha{ alpha }
        , gainBeta{ beta }
        , gainGamma{ gamma }
        , betaOverTs{ beta / Ts }
        , twoGammaOverTs2{ T{ 2 } * gamma / (Ts * Ts) }
    {}

    template<typename T, std::size_t Order>
    OPTIMIZE_FOR_SPEED T AlphaBetaFilter<T, Order>::Filter(T measuredPosition)
    {
        if (!initialized)
        {
            state[0] = measuredPosition;
            initialized = true;
            return measuredPosition;
        }

        T predicted0{};
        T predicted1{};
        T predicted2{};

        if constexpr (Order == 3)
        {
            predicted0 = state[0] + samplePeriod * state[1] + T{ 0.5 } * samplePeriod * samplePeriod * state[2];
            predicted1 = state[1] + samplePeriod * state[2];
            predicted2 = state[2];
        }
        else
        {
            predicted0 = state[0] + samplePeriod * state[1];
            predicted1 = state[1];
        }

        T residual{ measuredPosition - predicted0 };

        state[0] = predicted0 + gainAlpha * residual;
        state[1] = predicted1 + betaOverTs * residual;

        if constexpr (Order == 3)
        {
            state[2] = predicted2 + twoGammaOverTs2 * residual;
        }

        return state[0];
    }

    template<typename T, std::size_t Order>
    std::array<T, Order> AlphaBetaFilter<T, Order>::State() const
    {
        return state;
    }

    template<typename T, std::size_t Order>
    void AlphaBetaFilter<T, Order>::Reset(T position)
    {
        state = {};
        state[0] = position;
        initialized = false;
    }

    template<typename T, std::size_t Order>
    typename AlphaBetaFilter<T, Order>::Gains AlphaBetaFilter<T, Order>::GainsFromTrackingIndex(T lambda)
    {
        T r{ (T{ 4 } + lambda - math::Sqrt(T{ 8 } * lambda + lambda * lambda)) / T{ 4 } };
        T alpha{ T{ 1 } - r * r };
        T beta{ T{ 2 } * (T{ 2 } - alpha) - T{ 4 } * math::Sqrt(T{ 1 } - alpha) };
        return Gains{ alpha, beta };
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class AlphaBetaFilter<float, 2>;
    extern template class AlphaBetaFilter<float, 3>;
#endif
}
