#pragma once

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/optimization/StepOptimizer.hpp"

namespace optimization
{
    template<typename T, std::size_t N>
    class Adam
        : public StepOptimizer<T, N>
    {
        static_assert(std::is_floating_point_v<T>, "Adam supports floating-point types only");

    public:
        using Vector = typename StepOptimizer<T, N>::Vector;

        struct Parameters
        {
            T learningRate;
            T beta1{ T{ 0.9 } };
            T beta2{ T{ 0.999 } };
            T epsilon{ T{ 1e-8 } };
        };

        explicit Adam(const Parameters& params);

        void Step(Vector& theta, const Vector& gradient) override;
        void Reset() override;

    private:
        Parameters parameters;
        Vector firstMoment{};
        Vector secondMoment{};
        T beta1Power{ T{ 1 } };
        T beta2Power{ T{ 1 } };
    };

    template<typename T, std::size_t N>
    Adam<T, N>::Adam(const Parameters& params)
        : parameters{ params }
    {
        really_assert(params.learningRate > T{ 0 });
        really_assert(params.beta1 >= T{ 0 } && params.beta1 < T{ 1 });
        really_assert(params.beta2 >= T{ 0 } && params.beta2 < T{ 1 });
        really_assert(params.epsilon > T{ 0 });
    }

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED void Adam<T, N>::Step(Vector& theta, const Vector& gradient)
    {
        beta1Power *= parameters.beta1;
        beta2Power *= parameters.beta2;

        firstMoment = firstMoment * parameters.beta1 + gradient * (T{ 1 } - parameters.beta1);

        for (std::size_t i = 0; i < N; ++i)
            secondMoment[i] = parameters.beta2 * secondMoment[i] + (T{ 1 } - parameters.beta2) * gradient[i] * gradient[i];

        const T beta1Correction = T{ 1 } - beta1Power;
        const T beta2Correction = T{ 1 } - beta2Power;

        for (std::size_t i = 0; i < N; ++i)
        {
            const T mHat = firstMoment[i] / beta1Correction;
            const T vHat = secondMoment[i] / beta2Correction;
            theta[i] -= parameters.learningRate * mHat / (math::Sqrt(vHat) + parameters.epsilon);
        }
    }

    template<typename T, std::size_t N>
    void Adam<T, N>::Reset()
    {
        firstMoment = Vector{};
        secondMoment = Vector{};
        beta1Power = T{ 1 };
        beta2Power = T{ 1 };
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class Adam<float, 2>;
#endif
}

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif
