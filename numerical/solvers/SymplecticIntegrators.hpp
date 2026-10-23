#pragma once

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/solvers/SecondOrderSystem.hpp"
#include <cstddef>
#include <type_traits>

namespace solvers
{
    template<typename T, std::size_t Dimension>
    struct SecondOrderState
    {
        static_assert(std::is_floating_point_v<T>, "SecondOrderState supports floating-point types only");

        math::Vector<T, Dimension> q{};
        math::Vector<T, Dimension> qDot{};
    };

    template<typename T, std::size_t Dimension, std::size_t InputSize>
    class SemiImplicitEuler
    {
        static_assert(std::is_floating_point_v<T>, "SemiImplicitEuler supports floating-point types only");

    public:
        using State = SecondOrderState<T, Dimension>;
        using InputVector = math::Vector<T, InputSize>;

        SemiImplicitEuler(SecondOrderSystem<T, Dimension, InputSize>& system, T h);
        OPTIMIZE_FOR_SPEED State Step(const State& x, const InputVector& u);

    private:
        SecondOrderSystem<T, Dimension, InputSize>& system;
        T h;
    };

    template<typename T, std::size_t Dimension, std::size_t InputSize>
    class VelocityVerlet
    {
        static_assert(std::is_floating_point_v<T>, "VelocityVerlet supports floating-point types only");

    public:
        using State = SecondOrderState<T, Dimension>;
        using InputVector = math::Vector<T, InputSize>;

        VelocityVerlet(SecondOrderSystem<T, Dimension, InputSize>& system, T h);
        OPTIMIZE_FOR_SPEED State Step(const State& x, const InputVector& u);

    private:
        SecondOrderSystem<T, Dimension, InputSize>& system;
        T h;
    };

    template<typename T, std::size_t Dimension, std::size_t InputSize>
    SemiImplicitEuler<T, Dimension, InputSize>::SemiImplicitEuler(SecondOrderSystem<T, Dimension, InputSize>& system, T h)
        : system{ system }
        , h{ h }
    {
        really_assert(h > T{});
    }

    template<typename T, std::size_t Dimension, std::size_t InputSize>
    OPTIMIZE_FOR_SPEED auto SemiImplicitEuler<T, Dimension, InputSize>::Step(const State& x, const InputVector& u) -> State
    {
        const math::Vector<T, Dimension> qDot{ x.qDot + system.Acceleration(x.q, x.qDot, u) * h };
        return { x.q + qDot * h, qDot };
    }

    template<typename T, std::size_t Dimension, std::size_t InputSize>
    VelocityVerlet<T, Dimension, InputSize>::VelocityVerlet(SecondOrderSystem<T, Dimension, InputSize>& system, T h)
        : system{ system }
        , h{ h }
    {
        really_assert(h > T{});
    }

    template<typename T, std::size_t Dimension, std::size_t InputSize>
    OPTIMIZE_FOR_SPEED auto VelocityVerlet<T, Dimension, InputSize>::Step(const State& x, const InputVector& u) -> State
    {
        const T halfStep{ h / T{ 2 } };
        const math::Vector<T, Dimension> qDotHalf{ x.qDot + system.Acceleration(x.q, x.qDot, u) * halfStep };
        const math::Vector<T, Dimension> q{ x.q + qDotHalf * h };
        return { q, qDotHalf + system.Acceleration(q, qDotHalf, u) * halfStep };
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class SemiImplicitEuler<float, 1, 1>;
    extern template class VelocityVerlet<float, 1, 1>;
#endif
}
