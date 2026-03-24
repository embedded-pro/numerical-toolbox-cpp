#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/RecursiveBuffer.hpp"

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

namespace filters::passive
{
    template<typename QNumberType, std::size_t N>
    class Fir
    {
        static_assert(math::is_qnumber<QNumberType>::value ||
                          std::is_floating_point<QNumberType>::value,
            "Fir can only be instantiated with math::QNumber types.");

    public:
        explicit Fir(const math::RecursiveBuffer<QNumberType, N>& b) noexcept;

        QNumberType Filter(QNumberType input) noexcept;
        void Enable() noexcept;
        void Disable() noexcept;
        void Reset() noexcept;

    private:
        bool enabled = true;

        math::Index n;
        math::RecursiveBuffer<QNumberType, N> b;
        math::RecursiveBuffer<QNumberType, N> x;
    };

    ////    Implementation    ////

    template<typename QNumberType, std::size_t N>
    Fir<QNumberType, N>::Fir(const math::RecursiveBuffer<QNumberType, N>& b) noexcept
        : b(b)
    {
        Reset();
    }

    template<typename QNumberType, std::size_t N>
    OPTIMIZE_FOR_SPEED QNumberType Fir<QNumberType, N>::Filter(QNumberType input) noexcept
    {
        if (!enabled) [[unlikely]]
            return input;

        x.Update(input);

        QNumberType output{};
        for (std::size_t i = 0; i < N; ++i)
            output += b[n - i] * x[n - i];

        return output;
    }

    template<typename QNumberType, std::size_t N>
    void Fir<QNumberType, N>::Enable() noexcept
    {
        enabled = true;
    }

    template<typename QNumberType, std::size_t N>
    void Fir<QNumberType, N>::Disable() noexcept
    {
        enabled = false;
    }

    template<typename QNumberType, std::size_t N>
    void Fir<QNumberType, N>::Reset() noexcept
    {
        x.Reset();
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class Fir<float, 3>;
    extern template class Fir<math::Q15, 3>;
    extern template class Fir<math::Q31, 3>;
#endif
}
