#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/solvers/OdeSystem.hpp"
#include <cstddef>
#include <type_traits>

namespace solvers
{
    template<typename T, std::size_t StateSize, std::size_t InputSize>
    class RungeKutta4
    {
        static_assert(std::is_floating_point_v<T>, "RungeKutta4 supports floating-point types only");
        static_assert(StateSize > 0, "StateSize must be positive");

    public:
        using StateVector = math::Vector<T, StateSize>;
        using InputVector = math::Vector<T, InputSize>;

        RungeKutta4(OdeSystem<T, StateSize, InputSize>& system, T h);
        OPTIMIZE_FOR_SPEED StateVector Step(const StateVector& x, const InputVector& u, T t);

    private:
        OdeSystem<T, StateSize, InputSize>& system_;
        T h_;
    };

    template<typename T, std::size_t StateSize>
    class RungeKutta4<T, StateSize, 0>
    {
        static_assert(std::is_floating_point_v<T>, "RungeKutta4 supports floating-point types only");
        static_assert(StateSize > 0, "StateSize must be positive");

    public:
        using StateVector = math::Vector<T, StateSize>;

        RungeKutta4(OdeSystem<T, StateSize, 0>& system, T h);
        OPTIMIZE_FOR_SPEED StateVector Step(const StateVector& x, T t);

    private:
        OdeSystem<T, StateSize, 0>& system_;
        T h_;
    };

    // Implementation //

    template<typename T, std::size_t StateSize, std::size_t InputSize>
    RungeKutta4<T, StateSize, InputSize>::RungeKutta4(OdeSystem<T, StateSize, InputSize>& system, T h)
        : system_{ system }
        , h_{ h }
    {}

    template<typename T, std::size_t StateSize, std::size_t InputSize>
    typename RungeKutta4<T, StateSize, InputSize>::StateVector
        OPTIMIZE_FOR_SPEED
        RungeKutta4<T, StateSize, InputSize>::Step(const StateVector& x, const InputVector& u, T t)
    {
        StateVector k1{ system_.Derivative(x, u, t) };
        StateVector k2{ system_.Derivative(x + k1 * (h_ / T{ 2 }), u, t + h_ / T{ 2 }) };
        StateVector k3{ system_.Derivative(x + k2 * (h_ / T{ 2 }), u, t + h_ / T{ 2 }) };
        StateVector k4{ system_.Derivative(x + k3 * h_, u, t + h_) };
        return x + (k1 + k2 * T{ 2 } + k3 * T{ 2 } + k4) * (h_ / T{ 6 });
    }

    template<typename T, std::size_t StateSize>
    RungeKutta4<T, StateSize, 0>::RungeKutta4(OdeSystem<T, StateSize, 0>& system, T h)
        : system_{ system }
        , h_{ h }
    {}

    template<typename T, std::size_t StateSize>
    typename RungeKutta4<T, StateSize, 0>::StateVector
        OPTIMIZE_FOR_SPEED
        RungeKutta4<T, StateSize, 0>::Step(const StateVector& x, T t)
    {
        StateVector k1{ system_.Derivative(x, t) };
        StateVector k2{ system_.Derivative(x + k1 * (h_ / T{ 2 }), t + h_ / T{ 2 }) };
        StateVector k3{ system_.Derivative(x + k2 * (h_ / T{ 2 }), t + h_ / T{ 2 }) };
        StateVector k4{ system_.Derivative(x + k3 * h_, t + h_) };
        return x + (k1 + k2 * T{ 2 } + k3 * T{ 2 } + k4) * (h_ / T{ 6 });
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class RungeKutta4<float, 1, 0>;
#endif
}
