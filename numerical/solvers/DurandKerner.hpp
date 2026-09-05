#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "infra/util/BoundedVector.hpp"
#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/ComplexNumber.hpp"
#include "numerical/math/Math.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <numbers>
#include <span>
#include <type_traits>

namespace solvers
{
    template<typename T, std::size_t MaxOrder>
    class DurandKerner
    {
        static_assert(std::is_floating_point_v<T>,
            "DurandKerner only supports floating-point types");

    public:
        using Roots = typename infra::BoundedVector<math::Complex<T>>::template WithMaxSize<MaxOrder>;

        struct Result
        {
            Roots roots;
            bool converged{ false };
        };

        Result Solve(std::span<const T> coefficients,
            std::size_t maxIterations = 200, T tolerance = T(1e-6)) const;

    private:
        static math::Complex<T> EvaluatePolynomial(
            std::span<const T> coefficients, math::Complex<T> x);

        static math::Complex<T> ComputeDenominator(
            const Roots& roots, std::size_t r, std::size_t order);

        static bool Iterate(Roots& roots, std::span<const T> coefficients,
            std::size_t order, T tolerance);
    };

    ////    Implementation    ////

    template<typename T, std::size_t MaxOrder>
    math::Complex<T> DurandKerner<T, MaxOrder>::EvaluatePolynomial(
        std::span<const T> coefficients, math::Complex<T> x)
    {
        math::Complex<T> result(coefficients[0], T(0));
        for (std::size_t c = 1; c < coefficients.size(); ++c)
            result = result * x + math::Complex<T>(coefficients[c], T(0));
        return result;
    }

    template<typename T, std::size_t MaxOrder>
    math::Complex<T> DurandKerner<T, MaxOrder>::ComputeDenominator(
        const Roots& roots, std::size_t r, std::size_t order)
    {
        math::Complex<T> product(T(1), T(0));
        for (std::size_t j = 0; j < order; ++j)
        {
            if (j == r)
                continue;
            auto diff = roots[r] - roots[j];
            if (math::Abs(diff) > T(1e-15))
                product *= diff;
        }
        return product;
    }

    template<typename T, std::size_t MaxOrder>
    bool DurandKerner<T, MaxOrder>::Iterate(Roots& roots,
        std::span<const T> coefficients, std::size_t order, T tolerance)
    {
        bool converged = true;

        for (std::size_t r = 0; r < order; ++r)
        {
            auto pVal = EvaluatePolynomial(coefficients, roots[r]);
            auto denominator = ComputeDenominator(roots, r, order);
            auto correction = pVal / denominator;
            roots[r] -= correction;

            if (math::Abs(correction) > tolerance)
                converged = false;
        }

        return converged;
    }

    template<typename T, std::size_t MaxOrder>
    OPTIMIZE_FOR_SPEED typename DurandKerner<T, MaxOrder>::Result
    DurandKerner<T, MaxOrder>::Solve(std::span<const T> coefficients,
        std::size_t maxIterations, T tolerance) const
    {
        Result result;

        std::size_t lead = 0;
        while (lead < coefficients.size() && coefficients[lead] == T(0))
            ++lead;

        if (lead >= coefficients.size())
            return result;

        const std::size_t order = coefficients.size() - lead - 1;

        if (order > MaxOrder)
            return result;

        if (order == 0)
        {
            result.converged = true;
            return result;
        }

        std::array<T, MaxOrder + 1> monic{};
        const T leading = coefficients[lead];

        for (std::size_t i = 0; i <= order; ++i)
        {
            monic[i] = coefficients[lead + i] / leading;

            if (!math::IsFinite(monic[i]))
                return result;
        }

        const std::span<const T> normalized{ monic.data(), order + 1 };

        if (order == 1)
        {
            result.roots.emplace_back(-monic[1], T(0));
            result.converged = true;
            return result;
        }

        T radius = math::Pow(math::Abs(monic[order]), T(1) / static_cast<T>(order));
        if (radius < T(0.1))
            radius = T(1);

        for (std::size_t r = 0; r < order; ++r)
        {
            T angle = T(2) * std::numbers::pi_v<T> * static_cast<T>(r) / static_cast<T>(order) + T(0.4);
            result.roots.emplace_back(radius * math::Cos(angle), radius * math::Sin(angle));
        }

        for (std::size_t iter = 0; iter < maxIterations; ++iter)
            if (Iterate(result.roots, normalized, order, tolerance))
            {
                result.converged = true;
                break;
            }

        std::ranges::sort(result.roots,
            [](const math::Complex<T>& a, const math::Complex<T>& b)
            {
                if (a.Real() != b.Real())
                    return a.Real() < b.Real();
                return a.Imaginary() < b.Imaginary();
            });

        return result;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class DurandKerner<float, 10>;

    extern template class DurandKerner<double, 10>;
#endif
}
