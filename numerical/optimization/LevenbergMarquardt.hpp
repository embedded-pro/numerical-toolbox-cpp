#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/optimization/DampingPolicy.hpp"
#include "numerical/optimization/LeastSquaresProblem.hpp"
#include "numerical/solvers/LuDecomposition.hpp"
#include <cstddef>
#include <limits>
#include <optional>
#include <type_traits>

namespace optimization
{
    template<typename T>
    struct LevenbergMarquardtConfig
    {
        static_assert(std::is_floating_point_v<T>, "LevenbergMarquardtConfig supports floating-point types only");

        T tolerance{};
        T maxStep{ std::numeric_limits<T>::infinity() };
        std::size_t maxIterations{};
    };

    template<typename T, typename State>
    struct LevenbergMarquardtResult
    {
        static_assert(std::is_floating_point_v<T>, "LevenbergMarquardtResult supports floating-point types only");

        State x;
        T residualNorm{};
        std::size_t iterations{};
        bool converged{ false };
    };

    template<typename T, typename State, std::size_t Residuals, std::size_t Parameters>
    class LevenbergMarquardt
    {
        static_assert(std::is_floating_point_v<T>, "LevenbergMarquardt supports floating-point types only");

    public:
        using Problem = LeastSquaresProblem<T, State, Residuals, Parameters>;
        using Damping = DampingPolicy<T, Residuals, Parameters>;
        using Result = LevenbergMarquardtResult<T, State>;

        LevenbergMarquardt(const Problem& problem, const Damping& damping, const LevenbergMarquardtConfig<T>& config);

        OPTIMIZE_FOR_SPEED Result Solve(const State& initial) const;

    private:
        using ResidualVector = math::Vector<T, Residuals>;
        using StepVector = math::Vector<T, Parameters>;
        using Jacobian = math::Matrix<T, Residuals, Parameters>;

        std::optional<StepVector> DampedStep(const Jacobian& jacobian, const ResidualVector& residual) const;

        template<std::size_t Size>
        static T Norm(const math::Vector<T, Size>& v);

        const Problem& problem;
        const Damping& damping;
        LevenbergMarquardtConfig<T> config;
    };

    template<typename T, typename State, std::size_t Residuals, std::size_t Parameters>
    LevenbergMarquardt<T, State, Residuals, Parameters>::LevenbergMarquardt(const Problem& problem, const Damping& damping, const LevenbergMarquardtConfig<T>& config)
        : problem{ problem }
        , damping{ damping }
        , config{ config }
    {}

    template<typename T, typename State, std::size_t Residuals, std::size_t Parameters>
    OPTIMIZE_FOR_SPEED auto LevenbergMarquardt<T, State, Residuals, Parameters>::Solve(const State& initial) const -> Result
    {
        State x{ initial };
        for (std::size_t iteration = 0; iteration < config.maxIterations; ++iteration)
        {
            const ResidualVector residual{ problem.Residual(x) };
            const T residualNorm{ Norm(residual) };
            if (residualNorm < config.tolerance)
                return { x, residualNorm, iteration, true };

            auto step{ DampedStep(problem.Jacobian(x), residual) };
            if (!step)
                return { x, residualNorm, iteration, false };

            const T stepNorm{ Norm(*step) };
            if (stepNorm > config.maxStep)
                *step *= config.maxStep / stepNorm;

            x = problem.Retract(x, *step);
        }

        const T residualNorm{ Norm(problem.Residual(x)) };
        return { x, residualNorm, config.maxIterations, residualNorm < config.tolerance };
    }

    template<typename T, typename State, std::size_t Residuals, std::size_t Parameters>
    auto LevenbergMarquardt<T, State, Residuals, Parameters>::DampedStep(const Jacobian& jacobian, const ResidualVector& residual) const -> std::optional<StepVector>
    {
        const T lambdaSquared{ damping.DampingSquared(jacobian) };
        const auto transposed{ jacobian.Transpose() };

        if constexpr (Residuals <= Parameters)
        {
            solvers::LuDecomposition<T, Residuals> lu;
            if (!lu.Decompose(jacobian * transposed + math::SquareMatrix<T, Residuals>::Identity() * lambdaSquared))
                return std::nullopt;

            return transposed * lu.Solve(residual) * T{ -1 };
        }
        else
        {
            solvers::LuDecomposition<T, Parameters> lu;
            if (!lu.Decompose(transposed * jacobian + math::SquareMatrix<T, Parameters>::Identity() * lambdaSquared))
                return std::nullopt;

            return lu.Solve(transposed * residual) * T{ -1 };
        }
    }

    template<typename T, typename State, std::size_t Residuals, std::size_t Parameters>
    template<std::size_t Size>
    T LevenbergMarquardt<T, State, Residuals, Parameters>::Norm(const math::Vector<T, Size>& v)
    {
        T sum{};
        for (std::size_t i = 0; i < Size; ++i)
            sum += v[i] * v[i];

        return math::Sqrt(sum);
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class ConstantDamping<float, 2, 2>;
    extern template class LevenbergMarquardt<float, math::Vector<float, 2>, 2, 2>;
#endif
}
