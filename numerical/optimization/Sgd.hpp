#pragma once

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/optimization/StepOptimizer.hpp"

namespace optimization
{
    template<typename T, std::size_t N>
    class Sgd
        : public StepOptimizer<T, N>
    {
        static_assert(std::is_floating_point_v<T>, "Sgd supports floating-point types only");

    public:
        using Vector = typename StepOptimizer<T, N>::Vector;

        struct Parameters
        {
            T learningRate;
            T momentum{ T{ 0 } };
            bool nesterov{ false };
        };

        explicit Sgd(const Parameters& params);

        void Step(Vector& theta, const Vector& gradient) override;
        void Reset() override;

    private:
        Parameters parameters;
        Vector velocity{};
    };

    template<typename T, std::size_t N>
    Sgd<T, N>::Sgd(const Parameters& params)
        : parameters{ params }
    {
        really_assert(params.learningRate > T{ 0 });
        really_assert(params.momentum >= T{ 0 } && params.momentum < T{ 1 });
    }

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED void Sgd<T, N>::Step(Vector& theta, const Vector& gradient)
    {
        velocity = velocity * parameters.momentum + gradient;
        if (parameters.nesterov)
            theta = theta - (gradient + velocity * parameters.momentum) * parameters.learningRate;
        else
            theta = theta - velocity * parameters.learningRate;
    }

    template<typename T, std::size_t N>
    void Sgd<T, N>::Reset()
    {
        velocity = Vector{};
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class Sgd<float, 2>;
#endif
}

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif
