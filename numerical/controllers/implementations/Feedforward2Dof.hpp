#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/controllers/implementations/SaturationRateLimiter.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include <type_traits>

namespace controllers
{
    template<typename T>
    class Feedforward
    {
    public:
        static_assert(std::is_floating_point_v<T>, "Feedforward supports floating-point types");

        virtual ~Feedforward() = default;
        virtual T Evaluate(T reference) = 0;
        virtual void Reset() = 0;
    };

    template<typename T>
    class FeedbackLaw
    {
    public:
        static_assert(std::is_floating_point_v<T>, "FeedbackLaw supports floating-point types");

        virtual ~FeedbackLaw() = default;
        virtual T Process(T error) = 0;
        virtual void Reset() = 0;
    };

    template<typename T>
    class Feedforward2Dof
    {
    public:
        static_assert(std::is_floating_point_v<T>, "Feedforward2Dof supports floating-point types");

        Feedforward2Dof(Feedforward<T>& ff, FeedbackLaw<T>& fb, Saturation<T> clamp);

        OPTIMIZE_FOR_SPEED T Compute(T reference, T measurement);
        void Reset();

    private:
        Feedforward<T>& feedforward;
        FeedbackLaw<T>& feedback;
        Saturation<T> outputClamp;
    };

    template<typename T>
    Feedforward2Dof<T>::Feedforward2Dof(Feedforward<T>& ff, FeedbackLaw<T>& fb, Saturation<T> clamp)
        : feedforward{ ff }
        , feedback{ fb }
        , outputClamp{ clamp }
    {
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED T Feedforward2Dof<T>::Compute(T reference, T measurement)
    {
        T uFeedforward{ feedforward.Evaluate(reference) };
        T error{ reference - measurement };
        T uFeedback{ feedback.Process(error) };
        return outputClamp.Clamp(uFeedforward + uFeedback);
    }

    template<typename T>
    void Feedforward2Dof<T>::Reset()
    {
        feedforward.Reset();
        feedback.Reset();
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class Feedforward2Dof<float>;
#endif
}
