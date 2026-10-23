#pragma once

#include "numerical/math/CholeskyDecomposition.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/GivensRotation.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/TriangularSolve.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace solvers
{
    template<typename T, std::size_t Variables, std::size_t Equalities, std::size_t Inequalities>
    struct QuadraticProgram
    {
        static_assert(std::is_floating_point_v<T>, "QuadraticProgram supports floating-point types only");

        math::SquareMatrix<T, Variables> hessian{};
        math::Vector<T, Variables> gradient{};
        math::Matrix<T, Equalities, Variables> equalityMatrix{};
        math::Vector<T, Equalities> equalityVector{};
        math::Matrix<T, Inequalities, Variables> inequalityMatrix{};
        math::Vector<T, Inequalities> inequalityVector{};
    };

    template<typename T, std::size_t Variables, std::size_t Inequalities>
    struct QuadraticProgram<T, Variables, 0, Inequalities>
    {
        static_assert(std::is_floating_point_v<T>, "QuadraticProgram supports floating-point types only");

        math::SquareMatrix<T, Variables> hessian{};
        math::Vector<T, Variables> gradient{};
        math::Matrix<T, Inequalities, Variables> inequalityMatrix{};
        math::Vector<T, Inequalities> inequalityVector{};
    };

    template<typename T, std::size_t Variables, std::size_t Equalities>
    struct QuadraticProgram<T, Variables, Equalities, 0>
    {
        static_assert(std::is_floating_point_v<T>, "QuadraticProgram supports floating-point types only");

        math::SquareMatrix<T, Variables> hessian{};
        math::Vector<T, Variables> gradient{};
        math::Matrix<T, Equalities, Variables> equalityMatrix{};
        math::Vector<T, Equalities> equalityVector{};
    };

    enum class QpStatus : std::uint8_t
    {
        Optimal,
        Infeasible,
        NotPositiveDefinite,
        IterationLimit
    };

    template<typename T, std::size_t Variables, std::size_t Constraints>
    struct QpResult
    {
        static_assert(std::is_floating_point_v<T>, "QpResult supports floating-point types only");

        math::Vector<T, Variables> x{};
        math::Vector<T, Constraints> multipliers{};
        std::array<bool, Constraints> active{};
        std::size_t iterations{ 0 };
        QpStatus status{ QpStatus::IterationLimit };
    };

    template<typename T, std::size_t Variables, std::size_t Equalities, std::size_t Inequalities>
    class ActiveSetQp
    {
        static_assert(std::is_floating_point_v<T>, "ActiveSetQp supports floating-point types only");
        static_assert(Equalities + Inequalities > 0, "ActiveSetQp requires at least one constraint");
        static_assert(Equalities <= Variables, "ActiveSetQp requires Equalities <= Variables");

        static constexpr std::size_t Constraints{ Equalities + Inequalities };

    public:
        using Program = QuadraticProgram<T, Variables, Equalities, Inequalities>;
        using Result = QpResult<T, Variables, Constraints>;

        explicit ActiveSetQp(std::size_t maxIterations = 2 * Constraints + Variables, T feasibilityTolerance = T{ 1e-6f });

        OPTIMIZE_FOR_SPEED Result Solve(const Program& qp) const;

    private:
        using VariableVector = math::Vector<T, Variables>;
        using Basis = math::SquareMatrix<T, Variables>;

        enum class AddOutcome : std::uint8_t
        {
            Added,
            Infeasible,
            Stalled
        };

        struct State
        {
            Basis j{};
            Basis r{};
            VariableVector x{};
            std::array<T, Constraints> u{};
            std::array<T, Constraints> sign{};
            std::array<std::size_t, Variables> activeSet{};
            std::size_t activeCount{ 0 };
        };

        struct Direction
        {
            VariableVector d{};
            VariableVector z{};
            VariableVector r{};
            bool dependent{ false };
        };

        struct Blocking
        {
            T step{ std::numeric_limits<T>::infinity() };
            std::size_t position{ 0 };
        };

        static constexpr T dependencyTolerance{ T{ 1e-5f } };
        static constexpr T infinity{ std::numeric_limits<T>::infinity() };

        static VariableVector Normal(const Program& qp, std::size_t i);
        static T Bound(const Program& qp, std::size_t i);
        static bool Initialize(const Program& qp, State& state);
        bool SelectConstraint(const Program& qp, const State& state, std::size_t iteration, std::size_t& p) const;
        static AddOutcome AddConstraint(const Program& qp, State& state, std::size_t p);
        static Direction ComputeDirection(const State& state, const VariableVector& normal);
        static Blocking FindBlocking(const State& state, const Direction& direction);
        static void ActivateConstraint(State& state, VariableVector d, std::size_t p);
        static void DropConstraint(State& state, std::size_t position);
        static Result Finish(const State& state, std::size_t iterations, QpStatus status);

        std::size_t maxIterations;
        T feasibilityTolerance;
    };

    template<typename T, std::size_t Variables, std::size_t Equalities, std::size_t Inequalities>
    ActiveSetQp<T, Variables, Equalities, Inequalities>::ActiveSetQp(std::size_t maxIterations, T feasibilityTolerance)
        : maxIterations{ maxIterations }
        , feasibilityTolerance{ feasibilityTolerance }
    {}

    template<typename T, std::size_t Variables, std::size_t Equalities, std::size_t Inequalities>
    OPTIMIZE_FOR_SPEED auto ActiveSetQp<T, Variables, Equalities, Inequalities>::Solve(const Program& qp) const -> Result
    {
        State state{};
        if (!Initialize(qp, state))
            return Finish(state, 0, QpStatus::NotPositiveDefinite);

        for (std::size_t iteration = 0; iteration < maxIterations; ++iteration)
        {
            std::size_t p{ 0 };
            if (!SelectConstraint(qp, state, iteration, p))
                return Finish(state, iteration, QpStatus::Optimal);

            const AddOutcome outcome{ AddConstraint(qp, state, p) };
            if (outcome == AddOutcome::Infeasible)
                return Finish(state, iteration, QpStatus::Infeasible);
            if (outcome == AddOutcome::Stalled)
                return Finish(state, iteration, QpStatus::IterationLimit);
        }

        return Finish(state, maxIterations, QpStatus::IterationLimit);
    }

    template<typename T, std::size_t Variables, std::size_t Equalities, std::size_t Inequalities>
    auto ActiveSetQp<T, Variables, Equalities, Inequalities>::Normal(const Program& qp, std::size_t i) -> VariableVector
    {
        if constexpr (Inequalities == 0)
            return qp.equalityMatrix.template GetBlock<1, Variables>(i, 0).Transpose();
        else if constexpr (Equalities == 0)
            return qp.inequalityMatrix.template GetBlock<1, Variables>(i, 0).Transpose();
        else if (i < Equalities)
            return qp.equalityMatrix.template GetBlock<1, Variables>(i, 0).Transpose();
        else
            return qp.inequalityMatrix.template GetBlock<1, Variables>(i - Equalities, 0).Transpose();
    }

    template<typename T, std::size_t Variables, std::size_t Equalities, std::size_t Inequalities>
    T ActiveSetQp<T, Variables, Equalities, Inequalities>::Bound(const Program& qp, std::size_t i)
    {
        if constexpr (Inequalities == 0)
            return qp.equalityVector[i];
        else if constexpr (Equalities == 0)
            return qp.inequalityVector[i];
        else if (i < Equalities)
            return qp.equalityVector[i];
        else
            return qp.inequalityVector[i - Equalities];
    }

    template<typename T, std::size_t Variables, std::size_t Equalities, std::size_t Inequalities>
    bool ActiveSetQp<T, Variables, Equalities, Inequalities>::Initialize(const Program& qp, State& state)
    {
        const auto l{ math::CholeskyDecomposition<T, Variables>::TryFactor(qp.hessian) };
        if (!l)
            return false;

        for (std::size_t k = 0; k < Variables; ++k)
        {
            VariableVector unit{};
            unit[k] = T{ 1 };
            const VariableVector column{ math::SolveLowerTriangular(*l, unit) };
            for (std::size_t i = 0; i < Variables; ++i)
                state.j.at(k, i) = column[i];
        }

        state.x = state.j * (state.j.Transpose() * qp.gradient) * T{ -1 };
        state.sign.fill(T{ 1 });
        return true;
    }

    template<typename T, std::size_t Variables, std::size_t Equalities, std::size_t Inequalities>
    bool ActiveSetQp<T, Variables, Equalities, Inequalities>::SelectConstraint(const Program& qp, const State& state, std::size_t iteration, std::size_t& p) const
    {
        if (iteration < Equalities)
        {
            p = iteration;
            return true;
        }

        T worst{ infinity };
        for (std::size_t i = Equalities; i < Constraints; ++i)
        {
            bool isActive{ false };
            for (std::size_t l = 0; l < state.activeCount; ++l)
                isActive = isActive || state.activeSet[l] == i;

            const VariableVector normal{ Normal(qp, i) };
            const T slack{ (normal.Transpose() * state.x)[0] - Bound(qp, i) };
            const T scale{ math::Sqrt((normal.Transpose() * normal)[0]) };
            if (!isActive && slack < -feasibilityTolerance * scale && slack < worst)
            {
                worst = slack;
                p = i;
            }
        }

        return worst < infinity;
    }

    template<typename T, std::size_t Variables, std::size_t Equalities, std::size_t Inequalities>
    auto ActiveSetQp<T, Variables, Equalities, Inequalities>::AddConstraint(const Program& qp, State& state, std::size_t p) -> AddOutcome
    {
        if (p < Equalities && (Normal(qp, p).Transpose() * state.x)[0] > Bound(qp, p))
            state.sign[p] = T{ -1 };

        const VariableVector normal{ Normal(qp, p) * state.sign[p] };
        const T bound{ Bound(qp, p) * state.sign[p] };
        T multiplier{};

        for (std::size_t pass = 0; pass <= Variables + Constraints; ++pass)
        {
            const Direction direction{ ComputeDirection(state, normal) };

            const Blocking blocking{ FindBlocking(state, direction) };
            const T slack{ (normal.Transpose() * state.x)[0] - bound };
            const T curvature{ (direction.z.Transpose() * normal)[0] };
            const T fullStep{ !direction.dependent && curvature > T{} ? -slack / curvature : infinity };
            const T step{ std::min(blocking.step, fullStep) };
            if (step == infinity)
                return AddOutcome::Infeasible;

            for (std::size_t l = 0; l < state.activeCount; ++l)
                state.u[state.activeSet[l]] -= step * direction.r[l];
            multiplier += step;
            if (fullStep < infinity)
                state.x += direction.z * step;

            if (step == fullStep)
            {
                state.u[p] = multiplier;
                ActivateConstraint(state, direction.d, p);
                return AddOutcome::Added;
            }

            DropConstraint(state, blocking.position);
        }

        return AddOutcome::Stalled;
    }

    template<typename T, std::size_t Variables, std::size_t Equalities, std::size_t Inequalities>
    auto ActiveSetQp<T, Variables, Equalities, Inequalities>::ComputeDirection(const State& state, const VariableVector& normal) -> Direction
    {
        Direction direction{};
        direction.d = state.j.Transpose() * normal;

        const std::size_t q{ state.activeCount };
        T free{};
        T total{};
        for (std::size_t i = 0; i < Variables; ++i)
        {
            total += direction.d[i] * direction.d[i];
            if (i < q)
                continue;

            free += direction.d[i] * direction.d[i];
            for (std::size_t row = 0; row < Variables; ++row)
                direction.z[row] += state.j.at(row, i) * direction.d[i];
        }
        direction.dependent = free <= dependencyTolerance * dependencyTolerance * total;

        for (std::size_t n = q; n > 0; --n)
        {
            const std::size_t i{ n - 1 };
            T sum{ direction.d[i] };
            for (std::size_t k = i + 1; k < q; ++k)
                sum -= state.r.at(i, k) * direction.r[k];
            direction.r[i] = sum / state.r.at(i, i);
        }

        return direction;
    }

    template<typename T, std::size_t Variables, std::size_t Equalities, std::size_t Inequalities>
    auto ActiveSetQp<T, Variables, Equalities, Inequalities>::FindBlocking(const State& state, const Direction& direction) -> Blocking
    {
        Blocking blocking{};
        for (std::size_t l = 0; l < state.activeCount; ++l)
        {
            const std::size_t index{ state.activeSet[l] };
            if (index >= Equalities && direction.r[l] > T{} && state.u[index] / direction.r[l] < blocking.step)
                blocking = { state.u[index] / direction.r[l], l };
        }

        return blocking;
    }

    template<typename T, std::size_t Variables, std::size_t Equalities, std::size_t Inequalities>
    void ActiveSetQp<T, Variables, Equalities, Inequalities>::ActivateConstraint(State& state, VariableVector d, std::size_t p)
    {
        const std::size_t q{ state.activeCount };
        for (std::size_t i = Variables - 1; i > q; --i)
        {
            const auto rotation{ math::ComputeGivens(d[i - 1], d[i]) };
            math::ApplyGivens(rotation, d[i - 1], d[i]);
            for (std::size_t row = 0; row < Variables; ++row)
                math::ApplyGivens(rotation, state.j.at(row, i - 1), state.j.at(row, i));
        }

        for (std::size_t i = 0; i <= q; ++i)
            state.r.at(i, q) = d[i];

        state.activeSet[q] = p;
        ++state.activeCount;
    }

    template<typename T, std::size_t Variables, std::size_t Equalities, std::size_t Inequalities>
    void ActiveSetQp<T, Variables, Equalities, Inequalities>::DropConstraint(State& state, std::size_t position)
    {
        const std::size_t q{ state.activeCount };
        state.u[state.activeSet[position]] = T{};

        for (std::size_t c = position; c + 1 < q; ++c)
        {
            state.activeSet[c] = state.activeSet[c + 1];
            for (std::size_t row = 0; row < Variables; ++row)
                state.r.at(row, c) = state.r.at(row, c + 1);
        }
        for (std::size_t row = 0; row < Variables; ++row)
            state.r.at(row, q - 1) = T{};

        for (std::size_t i = position; i + 1 < q; ++i)
        {
            const auto rotation{ math::ComputeGivens(state.r.at(i, i), state.r.at(i + 1, i)) };
            for (std::size_t c = i; c < Variables; ++c)
                math::ApplyGivens(rotation, state.r.at(i, c), state.r.at(i + 1, c));
            for (std::size_t row = 0; row < Variables; ++row)
                math::ApplyGivens(rotation, state.j.at(row, i), state.j.at(row, i + 1));
        }

        --state.activeCount;
    }

    template<typename T, std::size_t Variables, std::size_t Equalities, std::size_t Inequalities>
    auto ActiveSetQp<T, Variables, Equalities, Inequalities>::Finish(const State& state, std::size_t iterations, QpStatus status) -> Result
    {
        Result result{};
        result.x = state.x;
        result.iterations = iterations;
        result.status = status;

        for (std::size_t l = 0; l < state.activeCount; ++l)
        {
            const std::size_t index{ state.activeSet[l] };
            result.active[index] = true;
            result.multipliers[index] = state.u[index] * state.sign[index];
        }

        return result;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class ActiveSetQp<float, 2, 1, 2>;
    extern template class ActiveSetQp<float, 2, 0, 4>;
    extern template class ActiveSetQp<float, 4, 2, 8>;
#endif
}
