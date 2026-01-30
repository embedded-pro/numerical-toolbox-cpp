#ifndef FILTERS_PASSIVE_IIR_HPP
#define FILTERS_PASSIVE_IIR_HPP

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/RecursiveBuffer.hpp"

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

namespace filters::passive
{
    template<typename QNumberType, std::size_t P, std::size_t Q>
    class Iir
    {
        static_assert(math::is_qnumber<QNumberType>::value ||
                          std::is_floating_point<QNumberType>::value,
            "Iir can only be instantiated with math::QNumber types.");

    public:
        Iir(const math::RecursiveBuffer<QNumberType, P>& b, const math::RecursiveBuffer<QNumberType, Q>& a) noexcept;

        QNumberType Filter(QNumberType input) noexcept;
        void Enable() noexcept;
        void Disable() noexcept;
        void Reset() noexcept;

    private:
        bool enabled = true;

        math::Index n;
        math::RecursiveBuffer<QNumberType, Q> a;
        math::RecursiveBuffer<QNumberType, P> b;
        math::RecursiveBuffer<QNumberType, Q> y;
        math::RecursiveBuffer<QNumberType, P> x;
    };

    ////    Implementation    ////

    template<typename QNumberType, std::size_t P, std::size_t Q>
    Iir<QNumberType, P, Q>::Iir(const math::RecursiveBuffer<QNumberType, P>& b, const math::RecursiveBuffer<QNumberType, Q>& a) noexcept
        : a(a)
        , b(b)
    {
        Reset();
    }

    template<typename QNumberType, std::size_t P, std::size_t Q>
    OPTIMIZE_FOR_SPEED QNumberType Iir<QNumberType, P, Q>::Filter(QNumberType input) noexcept
    {
        if (!enabled) [[unlikely]]
            return input;

        x.Update(input);

        QNumberType feedforward{};
        for (std::size_t i = 0; i < P; ++i)
            feedforward += b[n - i] * x[n - i];

        QNumberType feedback{};
        for (std::size_t i = 0; i < Q; ++i)
            feedback += a[n - i] * y[n - i];

        const auto output = feedforward + feedback;
        y.Update(output);

        return output;
    }

    template<typename QNumberType, std::size_t P, std::size_t Q>
    void Iir<QNumberType, P, Q>::Enable() noexcept
    {
        enabled = true;
    }

    template<typename QNumberType, std::size_t P, std::size_t Q>
    void Iir<QNumberType, P, Q>::Disable() noexcept
    {
        enabled = false;
    }

    template<typename QNumberType, std::size_t P, std::size_t Q>
    void Iir<QNumberType, P, Q>::Reset() noexcept
    {
        x.Reset();
        y.Reset();
    }
}

#endif
