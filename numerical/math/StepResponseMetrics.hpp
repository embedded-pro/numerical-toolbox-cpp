#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <type_traits>

namespace math
{
    template<typename T, std::size_t Size>
    [[nodiscard]] OPTIMIZE_FOR_SPEED T RiseTime(const Vector<T, Size>& response, T steady, T dt = T{ 1 })
    {
        static_assert(std::is_floating_point_v<T>, "RiseTime supports floating-point types only");

        const T lo{ T{ 0.1f } * steady };
        const T hi{ T{ 0.9f } * steady };

        std::size_t iLo{ 0 };
        bool foundLo{ false };

        for (std::size_t i = 0; i < Size; ++i)
        {
            const T val{ response.at(i, 0) };
            if (!foundLo && val >= lo)
            {
                iLo = i;
                foundLo = true;
            }
            if (foundLo && val >= hi)
                return static_cast<T>(i - iLo) * dt;
        }

        return static_cast<T>(Size - 1) * dt;
    }

    template<typename T, std::size_t Size>
    [[nodiscard]] OPTIMIZE_FOR_SPEED T SettlingTime(const Vector<T, Size>& response, T steady, T band = T{ 0.02f }, T dt = T{ 1 })
    {
        static_assert(std::is_floating_point_v<T>, "SettlingTime supports floating-point types only");

        const T absThreshold{ std::abs(steady) * band };
        std::size_t lastOutside{ 0 };
        bool anyOutside{ false };

        for (std::size_t i = 0; i < Size; ++i)
        {
            if (std::abs(response.at(i, 0) - steady) > absThreshold)
            {
                lastOutside = i;
                anyOutside = true;
            }
        }

        if (!anyOutside)
            return T{ 0 };

        return static_cast<T>(lastOutside + 1) * dt;
    }

    template<typename T, std::size_t Size>
    [[nodiscard]] OPTIMIZE_FOR_SPEED T PercentOvershoot(const Vector<T, Size>& response, T steady)
    {
        static_assert(std::is_floating_point_v<T>, "PercentOvershoot supports floating-point types only");

        if (steady == T{ 0 })
            return T{ 0 };

        T peak{ response.at(0, 0) };
        for (std::size_t i = 1; i < Size; ++i)
            peak = std::max(peak, response.at(i, 0));

        return T{ 100 } * (peak - steady) / steady;
    }

    template<typename T, std::size_t Size>
    [[nodiscard]] OPTIMIZE_FOR_SPEED T PeakTime(const Vector<T, Size>& response, T dt = T{ 1 })
    {
        static_assert(std::is_floating_point_v<T>, "PeakTime supports floating-point types only");

        std::size_t peakIdx{ 0 };
        T peakVal{ response.at(0, 0) };

        for (std::size_t i = 1; i < Size; ++i)
        {
            const T val{ response.at(i, 0) };
            if (val > peakVal)
            {
                peakVal = val;
                peakIdx = i;
            }
        }

        return static_cast<T>(peakIdx) * dt;
    }

    template<typename T, std::size_t Size, std::size_t TailSize = Size / 4 + 1>
    [[nodiscard]] OPTIMIZE_FOR_SPEED T SteadyStateError(const Vector<T, Size>& response, T reference)
    {
        static_assert(std::is_floating_point_v<T>, "SteadyStateError supports floating-point types only");

        T sum{ T{ 0 } };
        constexpr std::size_t tailStart{ Size - TailSize };

        for (std::size_t i = tailStart; i < Size; ++i)
            sum += response.at(i, 0);

        const T tailMean{ sum / static_cast<T>(TailSize) };
        return reference - tailMean;
    }
}
