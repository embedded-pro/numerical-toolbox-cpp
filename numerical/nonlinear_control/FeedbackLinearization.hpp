#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include <cstddef>
#include <type_traits>

namespace nonlinear_control
{
    template<typename T, std::size_t Dim>
    class ControlAffineModel
    {
        static_assert(std::is_floating_point_v<T>, "ControlAffineModel supports floating-point types");
        static_assert(Dim > 0, "ControlAffineModel requires Dim > 0");

    public:
        using StateVector = math::Vector<T, Dim>;
        using DecouplingMatrix = math::SquareMatrix<T, Dim>;

        virtual ~ControlAffineModel() = default;

        [[nodiscard]] virtual DecouplingMatrix DecouplingMatrixAt(const StateVector& x) const = 0;
        [[nodiscard]] virtual StateVector DriftTerm(const StateVector& x) const = 0;
    };

    template<typename T, std::size_t Dim>
    class FeedbackLinearization
    {
        static_assert(std::is_floating_point_v<T>, "FeedbackLinearization supports floating-point types");
        static_assert(Dim > 0, "FeedbackLinearization requires Dim > 0");

    public:
        using StateVector = math::Vector<T, Dim>;
        using GainMatrix = math::SquareMatrix<T, Dim>;

        FeedbackLinearization(const ControlAffineModel<T, Dim>& model, const GainMatrix& kp, const GainMatrix& kd);

        OPTIMIZE_FOR_SPEED StateVector ComputeInput(const StateVector& x, const StateVector& xDot,
            const StateVector& yd, const StateVector& ydDot, const StateVector& ydDdot);

    private:
        const ControlAffineModel<T, Dim>& model;
        GainMatrix kp;
        GainMatrix kd;
    };

    template<typename T, std::size_t Dim>
    FeedbackLinearization<T, Dim>::FeedbackLinearization(
        const ControlAffineModel<T, Dim>& model, const GainMatrix& kp, const GainMatrix& kd)
        : model{ model }
        , kp{ kp }
        , kd{ kd }
    {}

    template<typename T, std::size_t Dim>
    OPTIMIZE_FOR_SPEED typename FeedbackLinearization<T, Dim>::StateVector
    FeedbackLinearization<T, Dim>::ComputeInput(const StateVector& x, const StateVector& xDot,
        const StateVector& yd, const StateVector& ydDot, const StateVector& ydDdot)
    {
        const StateVector e{ yd - x };
        const StateVector eDot{ ydDot - xDot };
        const StateVector v{ ydDdot + kd * eDot + kp * e };
        const auto B{ model.DecouplingMatrixAt(x) };
        const StateVector a{ model.DriftTerm(x) };
        return B * v + a;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class FeedbackLinearization<float, 2>;
#endif
}
