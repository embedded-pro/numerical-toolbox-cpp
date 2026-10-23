#pragma once

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/motion_profiles/MotionProfileTypes.hpp"
#include <algorithm>
#include <array>
#include <cstdint>
#include <type_traits>

namespace motion_profiles
{
    enum class Degree : std::uint8_t
    {
        Cubic = 3,
        Quintic = 5
    };

    template<typename T>
    struct BoundaryConditions
    {
        static_assert(std::is_floating_point_v<T>, "BoundaryConditions supports floating-point types only");

        T q0{};
        T qf{};
        T v0{};
        T vf{};
        T a0{};
        T af{};
    };

    template<typename T>
    class PolynomialProfile
    {
        static_assert(std::is_floating_point_v<T>, "PolynomialProfile supports floating-point types only");

    public:
        PolynomialProfile(const BoundaryConditions<T>& bc, T tf, Degree degree);

        OPTIMIZE_FOR_SPEED TrajectoryState<T> Sample(T t) const;
        T Duration() const;
        void Reset(const BoundaryConditions<T>& bc, T tf);

    private:
        void SolveCubic(const BoundaryConditions<T>& bc);
        void SolveQuintic(const BoundaryConditions<T>& bc);

        std::array<T, 6> coeff{};
        T duration{};
        Degree degree;
    };

    template<typename T>
    PolynomialProfile<T>::PolynomialProfile(const BoundaryConditions<T>& bc, T tf, Degree degree)
        : degree{ degree }
    {
        Reset(bc, tf);
    }

    template<typename T>
    void PolynomialProfile<T>::Reset(const BoundaryConditions<T>& bc, T tf)
    {
        really_assert(tf > T{});

        duration = tf;
        if (degree == Degree::Cubic)
            SolveCubic(bc);
        else
            SolveQuintic(bc);
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED TrajectoryState<T> PolynomialProfile<T>::Sample(T t) const
    {
        t = std::min(std::max(t, T{}), duration);

        const auto& a{ coeff };
        return {
            a[0] + t * (a[1] + t * (a[2] + t * (a[3] + t * (a[4] + t * a[5])))),
            a[1] + t * (T{ 2 } * a[2] + t * (T{ 3 } * a[3] + t * (T{ 4 } * a[4] + t * T{ 5 } * a[5]))),
            T{ 2 } * a[2] + t * (T{ 6 } * a[3] + t * (T{ 12 } * a[4] + t * T{ 20 } * a[5])),
            T{ 6 } * a[3] + t * (T{ 24 } * a[4] + t * T{ 60 } * a[5])
        };
    }

    template<typename T>
    T PolynomialProfile<T>::Duration() const
    {
        return duration;
    }

    template<typename T>
    void PolynomialProfile<T>::SolveCubic(const BoundaryConditions<T>& bc)
    {
        const T tf{ duration };
        const T distance{ bc.qf - bc.q0 };

        coeff = {
            bc.q0,
            bc.v0,
            (T{ 3 } * distance - (T{ 2 } * bc.v0 + bc.vf) * tf) / (tf * tf),
            (T{ -2 } * distance + (bc.v0 + bc.vf) * tf) / (tf * tf * tf),
            T{},
            T{}
        };
    }

    template<typename T>
    void PolynomialProfile<T>::SolveQuintic(const BoundaryConditions<T>& bc)
    {
        const T tf{ duration };
        const T tf2{ tf * tf };
        const T tf3{ tf2 * tf };
        const T distance{ bc.qf - bc.q0 };

        coeff = {
            bc.q0,
            bc.v0,
            bc.a0 / T{ 2 },
            (T{ 20 } * distance - (T{ 8 } * bc.vf + T{ 12 } * bc.v0) * tf - (T{ 3 } * bc.a0 - bc.af) * tf2) / (T{ 2 } * tf3),
            (T{ -30 } * distance + (T{ 14 } * bc.vf + T{ 16 } * bc.v0) * tf + (T{ 3 } * bc.a0 - T{ 2 } * bc.af) * tf2) / (T{ 2 } * tf3 * tf),
            (T{ 12 } * distance - T{ 6 } * (bc.vf + bc.v0) * tf - (bc.a0 - bc.af) * tf2) / (T{ 2 } * tf3 * tf2)
        };
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class PolynomialProfile<float>;
#endif
}
