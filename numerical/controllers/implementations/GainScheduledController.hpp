#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include <array>
#include <cassert>
#include <cstddef>
#include <type_traits>

namespace controllers
{
    template<typename T, std::size_t GainSize>
    struct SchedulePoint
    {
        static_assert(std::is_floating_point_v<T>, "SchedulePoint supports floating-point types");

        T breakpoint;
        std::array<T, GainSize> gains;
    };

    template<typename T, std::size_t N, std::size_t GainSize>
    class GainScheduledController
    {
    public:
        static_assert(std::is_floating_point_v<T>, "GainScheduledController supports floating-point types");
        static_assert(N >= 2, "GainScheduledController requires at least 2 breakpoints");

        explicit GainScheduledController(std::array<SchedulePoint<T, GainSize>, N> scheduleTable);

        OPTIMIZE_FOR_SPEED const std::array<T, GainSize>& Schedule(T schedulingVariable);
        const std::array<T, GainSize>& ActiveGains() const;

    private:
        std::array<SchedulePoint<T, GainSize>, N> table;
        std::array<T, GainSize> active{};
    };

    template<typename T, std::size_t N, std::size_t GainSize>
    GainScheduledController<T, N, GainSize>::GainScheduledController(std::array<SchedulePoint<T, GainSize>, N> scheduleTable)
        : table{ scheduleTable }
    {
        for (std::size_t i{ 0 }; i < N - 1; ++i)
            assert(table[i].breakpoint < table[i + 1].breakpoint);
    }

    template<typename T, std::size_t N, std::size_t GainSize>
    OPTIMIZE_FOR_SPEED const std::array<T, GainSize>& GainScheduledController<T, N, GainSize>::Schedule(T schedulingVariable)
    {
        if (schedulingVariable <= table[0].breakpoint)
        {
            active = table[0].gains;
            return active;
        }

        if (schedulingVariable >= table[N - 1].breakpoint)
        {
            active = table[N - 1].gains;
            return active;
        }

        std::size_t i{ 0 };
        for (std::size_t k{ 0 }; k < N - 1; ++k)
        {
            if (table[k + 1].breakpoint > schedulingVariable)
            {
                i = k;
                break;
            }
        }

        T lo{ table[i].breakpoint };
        T hi{ table[i + 1].breakpoint };
        T w{ (schedulingVariable - lo) / (hi - lo) };

        for (std::size_t k{ 0 }; k < GainSize; ++k)
            active[k] = table[i].gains[k] + w * (table[i + 1].gains[k] - table[i].gains[k]);

        return active;
    }

    template<typename T, std::size_t N, std::size_t GainSize>
    const std::array<T, GainSize>& GainScheduledController<T, N, GainSize>::ActiveGains() const
    {
        return active;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class GainScheduledController<float, 2, 1>;
    extern template class GainScheduledController<float, 3, 1>;
    extern template class GainScheduledController<float, 3, 2>;
#endif
}
