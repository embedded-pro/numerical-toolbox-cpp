#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include <algorithm>
#include <type_traits>

namespace filters
{
    template<typename T>
    struct GravityTilt
    {
        static_assert(std::is_floating_point_v<T>, "GravityTilt supports floating-point types");

        T angle{};
        T trust{};
    };

    template<typename T>
    OPTIMIZE_FOR_SPEED GravityTilt<T> TiltFromGravity(T along, T across, T lateral, T gravity, T band)
    {
        static_assert(std::is_floating_point_v<T>, "TiltFromGravity supports floating-point types");

        const bool finite = math::IsFinite(along) && math::IsFinite(across) && math::IsFinite(lateral) && math::IsFinite(gravity) && math::IsFinite(band);
        if (!finite || !(gravity > T{}) || !(band > T{}))
            return {};

        const T magnitude{ math::Sqrt(along * along + across * across + lateral * lateral) };
        const T trust{ std::clamp(T{ 1 } - math::Abs(magnitude - gravity) / (band * gravity), T{}, T{ 1 }) };
        if (!(trust > T{}))
            return {};

        return { math::Atan2(along, across), trust };
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template GravityTilt<float> TiltFromGravity<float>(float, float, float, float, float);
#endif
}
