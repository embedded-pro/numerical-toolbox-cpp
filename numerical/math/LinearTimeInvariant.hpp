#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/QNumber.hpp"

namespace math
{
    // Discrete-time linear time-invariant state-space model:
    //   x_{k+1} = A x_k + B u_k
    //   y_k     = C x_k + D u_k
    //
    // Template args: <T, StateSize=n, InputSize=m, OutputSize=p>
    // Default OutputSize = StateSize (full-state output, C = I, D = 0).
    template<typename T,
        std::size_t StateSize,
        std::size_t InputSize,
        std::size_t OutputSize = StateSize>
    struct LinearTimeInvariant
    {
        static_assert(detail::is_supported_type_v<T>,
            "LinearTimeInvariant only supports float or QNumber types");
        static_assert(detail::is_valid_dimensions_v<StateSize, StateSize>,
            "StateSize must be positive");
        static_assert(detail::is_valid_dimensions_v<InputSize, InputSize>,
            "InputSize must be positive");
        static_assert(detail::is_valid_dimensions_v<OutputSize, OutputSize>,
            "OutputSize must be positive");

        using StateMatrix = SquareMatrix<T, StateSize>;
        using InputMatrix = Matrix<T, StateSize, InputSize>;
        using OutputMatrix = Matrix<T, OutputSize, StateSize>;
        using FeedthroughMatrix = Matrix<T, OutputSize, InputSize>;
        using StateVector = Vector<T, StateSize>;
        using InputVector = Vector<T, InputSize>;
        using OutputVector = Vector<T, OutputSize>;

        StateMatrix A{};
        InputMatrix B{};
        OutputMatrix C{};
        FeedthroughMatrix D{};

        // x_{k+1} = A x_k + B u_k
        ALWAYS_INLINE_HOT StateVector Step(const StateVector& x, const InputVector& u) const
        {
            return A * x + B * u;
        }

        // y_k = C x_k + D u_k
        ALWAYS_INLINE_HOT OutputVector Output(const StateVector& x, const InputVector& u) const
        {
            return C * x + D * u;
        }

        // Factory: C = I (OutputSize must equal StateSize), D = 0
        static LinearTimeInvariant WithFullStateOutput(
            const StateMatrix& stateTransition, const InputMatrix& inputMatrix)
        requires(OutputSize == StateSize)
        {
            LinearTimeInvariant lti;
            lti.A = stateTransition;
            lti.B = inputMatrix;
            lti.C = OutputMatrix::Identity();
            return lti;
        }

        // Factory: autonomous system (B = 0, D = 0)
        static LinearTimeInvariant Autonomous(
            const StateMatrix& stateTransition, const OutputMatrix& outputMatrix)
        {
            LinearTimeInvariant lti;
            lti.A = stateTransition;
            lti.C = outputMatrix;
            return lti;
        }
    };

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template struct LinearTimeInvariant<float, 2, 1>;
    extern template struct LinearTimeInvariant<float, 2, 1, 1>;
    extern template struct LinearTimeInvariant<float, 3, 1>;
    extern template struct LinearTimeInvariant<float, 4, 1>;
    extern template struct LinearTimeInvariant<float, 4, 1, 1>;
    extern template struct LinearTimeInvariant<float, 2, 2>;
    extern template struct LinearTimeInvariant<math::Q15, 2, 1>;
    extern template struct LinearTimeInvariant<math::Q15, 2, 1, 1>;
    extern template struct LinearTimeInvariant<math::Q31, 2, 1>;
    extern template struct LinearTimeInvariant<math::Q31, 2, 1, 1>;
#endif
}
