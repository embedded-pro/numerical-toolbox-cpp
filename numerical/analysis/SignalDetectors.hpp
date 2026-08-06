#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include <cstdint>
#include <type_traits>

namespace analysis
{
    template<typename T>
    class PeakHold
    {
        static_assert(std::is_floating_point_v<T>, "PeakHold supports floating-point types only");

    public:
        explicit PeakHold(T decay = T{ 1 });

        OPTIMIZE_FOR_SPEED T Update(T x);
        void Reset();

    private:
        T peak{ T{ 0 } };
        T decay;
    };

    template<typename T>
    class ZeroCrossingCounter
    {
        static_assert(std::is_floating_point_v<T>, "ZeroCrossingCounter supports floating-point types only");

    public:
        explicit ZeroCrossingCounter(T hysteresis = T{ 0 });

        OPTIMIZE_FOR_SPEED bool Update(T x);
        uint32_t Count() const;
        void Reset();

    private:
        T previous{ T{ 0 } };
        uint32_t count{ 0 };
        T hysteresis;
    };

    template<typename T>
    class RmsEnvelope
    {
        static_assert(std::is_floating_point_v<T>, "RmsEnvelope supports floating-point types only");

    public:
        explicit RmsEnvelope(T alpha);

        OPTIMIZE_FOR_SPEED T Update(T x);
        void Reset(T value = T{ 0 });

    private:
        T alpha;
        T meanSquare{ T{ 0 } };
    };

    /// Implementation ///

    template<typename T>
    PeakHold<T>::PeakHold(T decay)
        : decay{ decay }
    {}

    template<typename T>
    OPTIMIZE_FOR_SPEED T PeakHold<T>::Update(T x)
    {
        T m{ math::Abs(x) };
        peak = (m > peak * decay) ? m : peak * decay;
        return peak;
    }

    template<typename T>
    void PeakHold<T>::Reset()
    {
        peak = T{ 0 };
    }

    template<typename T>
    ZeroCrossingCounter<T>::ZeroCrossingCounter(T hysteresis)
        : hysteresis{ hysteresis }
    {}

    template<typename T>
    OPTIMIZE_FOR_SPEED bool ZeroCrossingCounter<T>::Update(T x)
    {
        bool crossed{ (previous < T{ 0 } ? x > T{ 0 } : x < T{ 0 }) && (math::Abs(x) > hysteresis) };
        if (crossed)
            ++count;
        previous = x;
        return crossed;
    }

    template<typename T>
    uint32_t ZeroCrossingCounter<T>::Count() const
    {
        return count;
    }

    template<typename T>
    void ZeroCrossingCounter<T>::Reset()
    {
        previous = T{ 0 };
        count = 0;
    }

    template<typename T>
    RmsEnvelope<T>::RmsEnvelope(T alpha)
        : alpha{ alpha }
    {}

    template<typename T>
    OPTIMIZE_FOR_SPEED T RmsEnvelope<T>::Update(T x)
    {
        meanSquare += alpha * (x * x - meanSquare);
        return math::Sqrt(meanSquare);
    }

    template<typename T>
    void RmsEnvelope<T>::Reset(T value)
    {
        meanSquare = value;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class PeakHold<float>;
    extern template class ZeroCrossingCounter<float>;
    extern template class RmsEnvelope<float>;
#endif
}
