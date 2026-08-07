#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include <cstddef>
#include <type_traits>

namespace nonlinear_control
{
    template<typename T, std::size_t Order>
    class StrictFeedbackModel
    {
        static_assert(std::is_floating_point_v<T>, "StrictFeedbackModel supports floating-point types");
        static_assert(Order > 0, "StrictFeedbackModel requires Order > 0");

    public:
        using StateVector = std::array<T, Order>;

        virtual ~StrictFeedbackModel() = default;

        virtual T Drift(std::size_t i, const StateVector& x) const = 0;
        virtual T Gain(std::size_t i, const StateVector& x) const = 0;
        virtual T VirtualDerivative(std::size_t i, const StateVector& x, T alpha) const = 0;
    };

    template<typename T, std::size_t Order>
    class BacksteppingControl
    {
        static_assert(std::is_floating_point_v<T>, "BacksteppingControl supports floating-point types");
        static_assert(Order > 0, "BacksteppingControl requires Order > 0");

    public:
        using StateVector = std::array<T, Order>;

        struct Reference
        {
            T value{};
            T derivative{};
        };

        BacksteppingControl(const StrictFeedbackModel<T, Order>& model,
            const std::array<T, Order>& gains);

        OPTIMIZE_FOR_SPEED T ComputeControl(const StateVector& x, const Reference& ref);
        void Reset();

    private:
        const StrictFeedbackModel<T, Order>& model;
        std::array<T, Order> gains;
        std::array<T, Order> z{};
    };

    template<typename T, std::size_t Order>
    BacksteppingControl<T, Order>::BacksteppingControl(
        const StrictFeedbackModel<T, Order>& model,
        const std::array<T, Order>& gains)
        : model{ model }
        , gains{ gains }
    {
        for (std::size_t i = 0; i < Order; ++i)
            really_assert(gains[i] > T{ 0 });
    }

    template<typename T, std::size_t Order>
    OPTIMIZE_FOR_SPEED T BacksteppingControl<T, Order>::ComputeControl(
        const StateVector& x, const Reference& ref)
    {
        z[0] = x[0] - ref.value;
        T alphaDot{ ref.derivative };

        T alpha{ T{ 0 } };
        for (std::size_t i = 0; i < Order; ++i)
        {
            const T f = model.Drift(i, x);
            const T g = model.Gain(i, x);
            really_assert(math::Abs(g) > T{ 0 });

            const T cross = (i == 0) ? T{ 0 } : model.Gain(i - 1, x) * z[i - 1];
            alpha = (alphaDot - f - gains[i] * z[i] - cross) / g;

            if (i < Order - 1)
            {
                z[i + 1] = x[i + 1] - alpha;
                alphaDot = model.VirtualDerivative(i, x, alpha);
            }
        }

        return alpha;
    }

    template<typename T, std::size_t Order>
    void BacksteppingControl<T, Order>::Reset()
    {
        z = {};
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class BacksteppingControl<float, 1>;
    extern template class BacksteppingControl<float, 2>;
#endif
}
