#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "infra/util/BoundedVector.hpp"
#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/ComplexNumber.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include <algorithm>
#include "numerical/math/Math.hpp"
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

        Roots Solve(std::span<const T> coefficients,
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
    OPTIMIZE_FOR_SPEED typename DurandKerner<T, MaxOrder>::Roots
    DurandKerner<T, MaxOrder>::Solve(std::span<const T> coefficients,
        std::size_t maxIterations, T tolerance) const
    {
        Roots roots;

        if (coefficients.empty())
            return roots;

        std::size_t order = coefficients.size() - 1;

        if (order == 0)
            return roots;

        really_assert(math::Abs(coefficients[0]) > T(0));
        really_assert(order <= MaxOrder);

        if (order == 1)
        {
            roots.emplace_back(-coefficients[1] / coefficients[0], T(0));
            return roots;
        }

        T radius = math::Pow(math::Abs(coefficients[order] / coefficients[0]),
            T(1) / static_cast<T>(order));
        if (radius < T(0.1))
            radius = T(1);

        for (std::size_t r = 0; r < order; ++r)
        {
            T angle = T(2) * std::numbers::pi_v<T> * static_cast<T>(r) / static_cast<T>(order) + T(0.4);
            roots.emplace_back(radius * math::Cos(angle), radius * math::Sin(angle));
        }

        for (std::size_t iter = 0; iter < maxIterations; ++iter)
        {
            if (Iterate(roots, coefficients, order, tolerance))
                break;
        }

        std::ranges::sort(roots,
            [](const math::Complex<T>& a, const math::Complex<T>& b)
            {
                if (math::Abs(a.Real() - b.Real()) > T(0.01))
                    return a.Real() < b.Real();
                return a.Imaginary() < b.Imaginary();
            });

        return roots;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class DurandKerner<float, 10>;

    extern template class DurandKerner<double, 10>;
#endif
}
