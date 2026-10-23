#pragma once

#include <type_traits>

namespace motion_profiles
{
    template<typename T>
    struct TrajectoryState
    {
        static_assert(std::is_floating_point_v<T>, "TrajectoryState supports floating-point types only");

        T position{};
        T velocity{};
        T acceleration{};
        T jerk{};

        bool operator==(const TrajectoryState& other) const = default;
    };

    template<typename T>
    struct MotionLimits
    {
        static_assert(std::is_floating_point_v<T>, "MotionLimits supports floating-point types only");

        T vMax{};
        T aMax{};
        T jMax{};
    };
}
