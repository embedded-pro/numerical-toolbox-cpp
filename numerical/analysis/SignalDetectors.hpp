#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include <cmath>
#include <cstdint>
#include <type_traits>

namespace analysis
{
    template<typename T>
    class PeakHold
    {
        static_assert(std::is_floating_point_v<T>, "PeakHold supports floating-point types only");

    public:
        explicit PeakHold(T decay = T{ 1 })
            : peak{ T{ 0 } }
            , decay{ decay }
        {}

        OPTIMIZE_FOR_SPEED T Update(T x)
        {
            T m{ std::abs(x) };
            peak = (m > peak * decay) ? m : peak * decay;
            return peak;
        }

        void Reset()
        {
            peak = T{ 0 };
        }

    private:
        T peak;
        T decay;
    };

    template<typename T>
    class ZeroCrossingCounter
    {
        static_assert(std::is_floating_point_v<T>, "ZeroCrossingCounter supports floating-point types only");

    public:
        explicit ZeroCrossingCounter(T hysteresis = T{ 0 })
            : previous{ T{ 0 } }
            , count{ 0 }
            , hysteresis{ hysteresis }
        {}

        OPTIMIZE_FOR_SPEED bool Update(T x)
        {
            bool crossed{ (previous < T{ 0 } ? x > T{ 0 } : x < T{ 0 }) && (std::abs(x) > hysteresis) };
            if (crossed)
                ++count;
            previous = x;
            return crossed;
        }

        uint32_t Count() const
        {
            return count;
        }

        void Reset()
        {
            previous = T{ 0 };
            count = 0;
        }

    private:
        T previous;
        uint32_t count;
        T hysteresis;
    };

    template<typename T>
    class RmsEnvelope
    {
        static_assert(std::is_floating_point_v<T>, "RmsEnvelope supports floating-point types only");

    public:
        explicit RmsEnvelope(T alpha)
            : alpha{ alpha }
            , meanSquare{ T{ 0 } }
        {}

        OPTIMIZE_FOR_SPEED T Update(T x)
        {
            meanSquare += alpha * (x * x - meanSquare);
            return std::sqrt(meanSquare);
        }

        void Reset(T value = T{ 0 })
        {
            meanSquare = value;
        }

    private:
        T alpha;
        T meanSquare;
    };

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class PeakHold<float>;
    extern template class ZeroCrossingCounter<float>;
    extern template class RmsEnvelope<float>;
#endif
}
