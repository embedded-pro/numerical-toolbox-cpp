#pragma once

#include "infra/util/BoundedVector.hpp"
#include "infra/util/MemoryRange.hpp"
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <numbers>
#include <type_traits>

namespace solvers
{
    template<typename T, std::size_t MaxOrder>
    class DurandKerner
    {
        static_assert(std::is_floating_point_v<T>,
            "DurandKerner only supports floating-point types");

    public:
        using Roots = typename infra::BoundedVector<std::complex<T>>::template WithMaxSize<MaxOrder>;

        Roots Solve(infra::MemoryRange<const T> coefficients,
            std::size_t maxIterations = 200, T tolerance = T(1e-6)) const;
    };

    ////    Implementation    ////

    template<typename T, std::size_t MaxOrder>
    typename DurandKerner<T, MaxOrder>::Roots
    DurandKerner<T, MaxOrder>::Solve(infra::MemoryRange<const T> coefficients,
        std::size_t maxIterations, T tolerance) const
    {
        std::size_t order = coefficients.size() - 1;
        Roots roots;

        if (order == 0)
            return roots;

        if (order == 1)
        {
            roots.emplace_back(-coefficients[1] / coefficients[0], T(0));
            return roots;
        }

        T radius = std::pow(std::abs(coefficients[order] / coefficients[0]),
            T(1) / static_cast<T>(order));
        if (radius < T(0.1))
            radius = T(1);

        for (std::size_t r = 0; r < order; ++r)
        {
            T angle = T(2) * std::numbers::pi_v<T> * static_cast<T>(r) / static_cast<T>(order) + T(0.4);
            roots.emplace_back(radius * std::cos(angle), radius * std::sin(angle));
        }

        for (std::size_t iter = 0; iter < maxIterations; ++iter)
        {
            bool converged = true;

            for (std::size_t r = 0; r < order; ++r)
            {
                std::complex<T> pVal(coefficients[0], T(0));
                for (std::size_t c = 1; c < coefficients.size(); ++c)
                    pVal = pVal * roots[r] + std::complex<T>(coefficients[c], T(0));

                std::complex<T> product(T(1), T(0));
                for (std::size_t j = 0; j < order; ++j)
                {
                    if (j != r)
                    {
                        auto diff = roots[r] - roots[j];
                        if (std::abs(diff) > T(1e-15))
                            product *= diff;
                    }
                }

                auto correction = pVal / product;
                roots[r] -= correction;

                if (std::abs(correction) > tolerance)
                    converged = false;
            }

            if (converged)
                break;
        }

        std::sort(roots.begin(), roots.end(),
            [](const std::complex<T>& a, const std::complex<T>& b)
            {
                if (std::abs(a.real() - b.real()) > T(0.01))
                    return a.real() < b.real();
                return a.imag() < b.imag();
            });

        return roots;
    }
}
