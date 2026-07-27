#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include <type_traits>

namespace controllers
{
    template<typename T>
    struct LeadLagParameters
    {
        static_assert(std::is_floating_point_v<T>, "LeadLagParameters supports floating-point types");

        T gain;
        T zero;
        T pole;
        T sampleTime;
    };

    template<typename T>
    class LeadLagCompensator
    {
        static_assert(std::is_floating_point_v<T>, "LeadLagCompensator supports floating-point types");

    public:
        explicit LeadLagCompensator(LeadLagParameters<T> p);

        OPTIMIZE_FOR_SPEED T Compute(T input);

        void Reset(T value = T{ 0 });

    private:
        T b0{};
        T b1{};
        T a1{};
        T prevInput{};
        T prevOutput{};
    };

    template<typename T>
    LeadLagCompensator<T>::LeadLagCompensator(LeadLagParameters<T> p)
    {
        T c{ T{ 2 } / p.sampleTime };
        T n0{ p.gain * (c + p.zero) };
        T n1{ p.gain * (p.zero - c) };
        T d0{ c + p.pole };
        T d1{ p.pole - c };
        b0 = n0 / d0;
        b1 = n1 / d0;
        a1 = d1 / d0;
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED T LeadLagCompensator<T>::Compute(T input)
    {
        T output{ b0 * input + b1 * prevInput - a1 * prevOutput };
        prevInput = input;
        prevOutput = output;
        return output;
    }

    template<typename T>
    void LeadLagCompensator<T>::Reset(T value)
    {
        prevInput = value;
        prevOutput = value;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class LeadLagCompensator<float>;
#endif
}
