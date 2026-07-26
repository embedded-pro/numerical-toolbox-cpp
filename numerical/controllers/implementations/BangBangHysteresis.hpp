#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include <type_traits>

namespace controllers
{
    enum class RelayState
    {
        Low,
        High
    };

    template<typename T>
    class BangBangHysteresis
    {
    public:
        static_assert(std::is_floating_point_v<T>, "BangBangHysteresis supports floating-point types");

        BangBangHysteresis(T lowThreshold, T highThreshold, T outputLow, T outputHigh);

        OPTIMIZE_FOR_SPEED T Update(T measurement);
        void Reset(RelayState initial = RelayState::Low);
        RelayState State() const;

    private:
        T lowThreshold;
        T highThreshold;
        T outputLow;
        T outputHigh;
        RelayState state{ RelayState::Low };
    };

    template<typename T>
    BangBangHysteresis<T>::BangBangHysteresis(T lowThreshold, T highThreshold, T outputLow, T outputHigh)
        : lowThreshold{ lowThreshold }
        , highThreshold{ highThreshold }
        , outputLow{ outputLow }
        , outputHigh{ outputHigh }
    {
        really_assert(highThreshold > lowThreshold);
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED T BangBangHysteresis<T>::Update(T measurement)
    {
        if (state == RelayState::Low && measurement >= highThreshold)
            state = RelayState::High;
        else if (state == RelayState::High && measurement <= lowThreshold)
            state = RelayState::Low;

        return (state == RelayState::High) ? outputHigh : outputLow;
    }

    template<typename T>
    void BangBangHysteresis<T>::Reset(RelayState initial)
    {
        state = initial;
    }

    template<typename T>
    RelayState BangBangHysteresis<T>::State() const
    {
        return state;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class BangBangHysteresis<float>;
#endif
}
