#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/LinearTimeInvariant.hpp"
#include "numerical/solvers/GaussianElimination.hpp"
#include <array>
#include <cstddef>
#include <type_traits>

namespace controllers
{
    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>
    class LuenbergerObserver
    {
        static_assert(std::is_floating_point_v<T>, "LuenbergerObserver supports floating-point types");
        static_assert(StateSize > 0, "StateSize must be positive");
        static_assert(InputSize > 0, "InputSize must be positive");
        static_assert(OutputSize > 0, "OutputSize must be positive");

    public:
        using Plant = math::LinearTimeInvariant<T, StateSize, InputSize, OutputSize>;
        using StateMatrix = math::SquareMatrix<T, StateSize>;
        using GainMatrix = math::Matrix<T, StateSize, OutputSize>;
        using StateVector = math::Vector<T, StateSize>;
        using InputVector = math::Vector<T, InputSize>;
        using OutputVector = math::Vector<T, OutputSize>;

        LuenbergerObserver(const Plant& plant, const GainMatrix& observerGain);

        static GainMatrix AckermannGain(const Plant& plant, const std::array<T, StateSize>& desiredPoles);

        OPTIMIZE_FOR_SPEED StateVector Update(const InputVector& u, const OutputVector& y);

        [[nodiscard]] const StateVector& Estimate() const;

        void Reset(const StateVector& x0);

    private:
        static StateMatrix BuildObservabilityMatrix(const StateMatrix& A, const math::Matrix<T, OutputSize, StateSize>& C);
        static StateMatrix EvaluateCharacteristicPoly(const StateMatrix& A, const std::array<T, StateSize>& poles);

        Plant plant;
        GainMatrix L;
        StateVector xhat{};
    };

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>
    LuenbergerObserver<T, StateSize, InputSize, OutputSize>::LuenbergerObserver(
        const Plant& plantModel, const GainMatrix& observerGain)
        : plant{ plantModel }
        , L{ observerGain }
    {}

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>
    OPTIMIZE_FOR_SPEED
        typename LuenbergerObserver<T, StateSize, InputSize, OutputSize>::StateVector
        LuenbergerObserver<T, StateSize, InputSize, OutputSize>::Update(
            const InputVector& u, const OutputVector& y)
    {
        OutputVector yhat = plant.C * xhat + plant.D * u;
        OutputVector innovation = y - yhat;
        xhat = plant.A * xhat + plant.B * u + L * innovation;
        return xhat;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>
    const typename LuenbergerObserver<T, StateSize, InputSize, OutputSize>::StateVector&
    LuenbergerObserver<T, StateSize, InputSize, OutputSize>::Estimate() const
    {
        return xhat;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>
    void LuenbergerObserver<T, StateSize, InputSize, OutputSize>::Reset(const StateVector& x0)
    {
        xhat = x0;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>
    typename LuenbergerObserver<T, StateSize, InputSize, OutputSize>::StateMatrix
    LuenbergerObserver<T, StateSize, InputSize, OutputSize>::BuildObservabilityMatrix(
        const StateMatrix& A, const math::Matrix<T, OutputSize, StateSize>& C)
    {
        static_assert(OutputSize == 1, "Ackermann's formula requires SISO output (OutputSize == 1)");
        StateMatrix O{};
        math::Matrix<T, OutputSize, StateSize> CAk = C;
        for (std::size_t k = 0; k < StateSize; ++k)
        {
            for (std::size_t col = 0; col < StateSize; ++col)
                O.at(k, col) = CAk.at(0, col);
            CAk = CAk * A;
        }
        return O;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>
    typename LuenbergerObserver<T, StateSize, InputSize, OutputSize>::StateMatrix
    LuenbergerObserver<T, StateSize, InputSize, OutputSize>::EvaluateCharacteristicPoly(
        const StateMatrix& A, const std::array<T, StateSize>& poles)
    {
        StateMatrix result = StateMatrix::Identity();
        for (std::size_t i = 0; i < StateSize; ++i)
        {
            StateMatrix shifted = A;
            for (std::size_t r = 0; r < StateSize; ++r)
                shifted.at(r, r) = shifted.at(r, r) - poles[i];
            result = result * shifted;
        }
        return result;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>
    typename LuenbergerObserver<T, StateSize, InputSize, OutputSize>::GainMatrix
    LuenbergerObserver<T, StateSize, InputSize, OutputSize>::AckermannGain(
        const Plant& plant, const std::array<T, StateSize>& desiredPoles)
    {
        static_assert(OutputSize == 1, "Ackermann's formula requires SISO output (OutputSize == 1)");

        StateMatrix O = BuildObservabilityMatrix(plant.A, plant.C);
        StateMatrix phi = EvaluateCharacteristicPoly(plant.A, desiredPoles);

        StateVector eLast{};
        eLast.at(StateSize - 1, 0) = T(1);

        StateVector OinvEn = solvers::SolveSystem<T, StateSize, 1>(O, eLast);

        StateVector Lgain = phi * OinvEn;

        GainMatrix result{};
        for (std::size_t i = 0; i < StateSize; ++i)
            result.at(i, 0) = Lgain.at(i, 0);
        return result;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class LuenbergerObserver<float, 2, 1, 1>;
    extern template class LuenbergerObserver<float, 3, 1, 1>;
    extern template class LuenbergerObserver<float, 2, 2, 1>;
#endif
}
