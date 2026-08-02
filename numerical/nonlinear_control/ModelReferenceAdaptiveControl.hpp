#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/LinearTimeInvariant.hpp"
#include "numerical/math/Matrix.hpp"
#include <cstddef>
#include <type_traits>

namespace nonlinear_control
{
    enum class AdaptationLaw
    {
        MitRule,
        Lyapunov
    };

    template<typename T, std::size_t StateSize, std::size_t InputSize>
    class ModelReferenceAdaptiveControl
    {
        static_assert(std::is_floating_point_v<T>,
            "ModelReferenceAdaptiveControl supports floating-point types");
        static_assert(StateSize > 0, "ModelReferenceAdaptiveControl requires StateSize > 0");
        static_assert(InputSize > 0, "ModelReferenceAdaptiveControl requires InputSize > 0");

    public:
        using StateVector = math::Vector<T, StateSize>;
        using InputVector = math::Vector<T, InputSize>;
        using FeedbackMatrix = math::Matrix<T, InputSize, StateSize>;
        using FeedforwardMatrix = math::Matrix<T, InputSize, InputSize>;
        using ReferenceModel = math::LinearTimeInvariant<T, StateSize, InputSize, StateSize>;

        ModelReferenceAdaptiveControl(const ReferenceModel& referenceModel,
            T gamma, T signB, AdaptationLaw law);

        OPTIMIZE_FOR_SPEED InputVector ComputeControl(
            const StateVector& x, const InputVector& r, T dt);

        void Reset();

        [[nodiscard]] const StateVector& GetReferenceState() const
        {
            return xm;
        }

        [[nodiscard]] const FeedbackMatrix& GetThetaX() const
        {
            return thetaX;
        }

        [[nodiscard]] const FeedforwardMatrix& GetThetaR() const
        {
            return thetaR;
        }

    private:
        const ReferenceModel& reference;
        StateVector xm{};
        FeedbackMatrix thetaX{};
        FeedforwardMatrix thetaR{};
        T gamma;
        T signB;
        AdaptationLaw law;
    };

    template<typename T, std::size_t StateSize, std::size_t InputSize>
    ModelReferenceAdaptiveControl<T, StateSize, InputSize>::ModelReferenceAdaptiveControl(
        const ReferenceModel& referenceModel, T gamma, T signB, AdaptationLaw law)
        : reference{ referenceModel }
        , gamma{ gamma }
        , signB{ signB }
        , law{ law }
    {}

    template<typename T, std::size_t StateSize, std::size_t InputSize>
    OPTIMIZE_FOR_SPEED typename ModelReferenceAdaptiveControl<T, StateSize, InputSize>::InputVector
    ModelReferenceAdaptiveControl<T, StateSize, InputSize>::ComputeControl(
        const StateVector& x, const InputVector& r, T dt)
    {
        xm = xm + (reference.A * xm + reference.B * r) * dt;

        const StateVector e{ x - xm };

        const InputVector u{ thetaX * x + thetaR * r };

        const T scale{ gamma * signB * dt };
        for (std::size_t i = 0; i < InputSize; ++i)
        {
            for (std::size_t j = 0; j < StateSize; ++j)
                thetaX.at(i, j) -= scale * e.at(i, 0) * x.at(j, 0);

            for (std::size_t j = 0; j < InputSize; ++j)
                thetaR.at(i, j) -= scale * e.at(i, 0) * r.at(j, 0);
        }

        return u;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize>
    void ModelReferenceAdaptiveControl<T, StateSize, InputSize>::Reset()
    {
        xm = StateVector{};
        thetaX = FeedbackMatrix{};
        thetaR = FeedforwardMatrix{};
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class ModelReferenceAdaptiveControl<float, 1, 1>;
#endif
}
