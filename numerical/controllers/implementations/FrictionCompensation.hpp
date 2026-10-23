#pragma once

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <type_traits>

namespace controllers
{
    template<typename T>
    struct FrictionParameters
    {
        static_assert(std::is_floating_point_v<T>, "FrictionParameters supports floating-point types only");

        T coulomb{};
        T stiction{};
        T viscous{};
        T stribeckVelocity{};
        T stribeckShape{};
        T smoothingVelocity{};
        T maxCompensation{};
    };

    template<typename T, std::size_t Axes>
    class FrictionCompensation
    {
        static_assert(std::is_floating_point_v<T>, "FrictionCompensation supports floating-point types only");
        static_assert(Axes > 0, "FrictionCompensation requires at least one axis");

    public:
        using Parameters = std::array<FrictionParameters<T>, Axes>;
        using AxisVector = math::Vector<T, Axes>;

        explicit FrictionCompensation(const Parameters& params);

        OPTIMIZE_FOR_SPEED AxisVector Compute(const AxisVector& velocity) const;
        OPTIMIZE_FOR_SPEED static T AxisTorque(const FrictionParameters<T>& p, T velocity);

    private:
        Parameters params;
    };

    template<typename T, std::size_t Axes>
    FrictionCompensation<T, Axes>::FrictionCompensation(const Parameters& params)
        : params{ params }
    {
        for (const auto& p : params)
        {
            really_assert(p.stiction >= p.coulomb);
            really_assert(p.stribeckVelocity > T{});
            really_assert(p.smoothingVelocity > T{});
            really_assert(p.maxCompensation > T{});
        }
    }

    template<typename T, std::size_t Axes>
    OPTIMIZE_FOR_SPEED auto FrictionCompensation<T, Axes>::Compute(const AxisVector& velocity) const -> AxisVector
    {
        AxisVector torque{};
        for (std::size_t i = 0; i < Axes; ++i)
            torque[i] = AxisTorque(params[i], velocity[i]);

        return torque;
    }

    template<typename T, std::size_t Axes>
    OPTIMIZE_FOR_SPEED T FrictionCompensation<T, Axes>::AxisTorque(const FrictionParameters<T>& p, T velocity)
    {
        const T fall{ math::Exp(-math::Pow(math::Abs(velocity) / p.stribeckVelocity, p.stribeckShape)) };
        const T level{ p.coulomb + (p.stiction - p.coulomb) * fall };
        const T torque{ level * math::Tanh(velocity / p.smoothingVelocity) + p.viscous * velocity };

        return std::min(std::max(torque, -p.maxCompensation), p.maxCompensation);
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class FrictionCompensation<float, 1>;
    extern template class FrictionCompensation<float, 3>;
#endif
}
