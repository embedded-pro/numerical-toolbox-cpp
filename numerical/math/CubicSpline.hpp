#pragma once

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/TridiagonalSolver.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace math
{
    enum class SplineEnd : std::uint8_t
    {
        Clamped,
        Natural
    };

    template<typename T, std::size_t Dimension>
    struct SplineBoundary
    {
        static_assert(std::is_floating_point_v<T>, "SplineBoundary supports floating-point types only");

        SplineEnd condition{ SplineEnd::Clamped };
        Vector<T, Dimension> startSlope{};
        Vector<T, Dimension> endSlope{};
    };

    template<typename T, std::size_t Dimension>
    struct SplinePoint
    {
        static_assert(std::is_floating_point_v<T>, "SplinePoint supports floating-point types only");

        Vector<T, Dimension> value{};
        Vector<T, Dimension> firstDerivative{};
        Vector<T, Dimension> secondDerivative{};
    };

    template<typename T, std::size_t Dimension, std::size_t MaxKnots>
    class CubicSpline
    {
        static_assert(std::is_floating_point_v<T>, "CubicSpline supports floating-point types only");
        static_assert(MaxKnots >= 2, "CubicSpline requires MaxKnots >= 2");

    public:
        using Value = Vector<T, Dimension>;
        using Knots = std::array<T, MaxKnots>;
        using Values = std::array<Value, MaxKnots>;

        bool Fit(const Knots& x, const Values& y, std::size_t n, const SplineBoundary<T, Dimension>& boundary = {});
        OPTIMIZE_FOR_SPEED SplinePoint<T, Dimension> Evaluate(T x) const;
        T First() const;
        T Last() const;

    private:
        using Band = typename TridiagonalSolver<T, MaxKnots>::Band;

        static bool IsStrictlyIncreasing(const Knots& x, std::size_t n);
        bool FactorSystem(TridiagonalSolver<T, MaxKnots>& solver, SplineEnd condition) const;
        Band RightHandSide(std::size_t channel, const SplineBoundary<T, Dimension>& boundary) const;
        std::size_t Interval(T x) const;

        Knots knots{};
        Values values{};
        Values secondDerivatives{};
        Knots inverseSpan{};
        std::size_t count{ 0 };
    };

    template<typename T, std::size_t Dimension, std::size_t MaxKnots>
    bool CubicSpline<T, Dimension, MaxKnots>::Fit(const Knots& x, const Values& y, std::size_t n, const SplineBoundary<T, Dimension>& boundary)
    {
        count = 0;
        if (n < 2 || n > MaxKnots || !IsStrictlyIncreasing(x, n))
            return false;

        knots = x;
        values = y;
        for (std::size_t k = 0; k + 1 < n; ++k)
            inverseSpan[k] = T{ 1 } / (x[k + 1] - x[k]);
        count = n;

        TridiagonalSolver<T, MaxKnots> solver;
        if (!FactorSystem(solver, boundary.condition))
        {
            count = 0;
            return false;
        }

        for (std::size_t j = 0; j < Dimension; ++j)
        {
            const Band m{ solver.Solve(RightHandSide(j, boundary)) };
            for (std::size_t k = 0; k < count; ++k)
                secondDerivatives[k][j] = m[k];
        }

        return true;
    }

    template<typename T, std::size_t Dimension, std::size_t MaxKnots>
    OPTIMIZE_FOR_SPEED SplinePoint<T, Dimension> CubicSpline<T, Dimension, MaxKnots>::Evaluate(T x) const
    {
        really_assert(count >= 2);

        x = std::min(std::max(x, knots[0]), knots[count - 1]);
        const std::size_t k{ Interval(x) };
        const T tau{ x - knots[k] };
        const T h{ knots[k + 1] - knots[k] };
        const T invH{ inverseSpan[k] };

        SplinePoint<T, Dimension> point{};
        for (std::size_t j = 0; j < Dimension; ++j)
        {
            const T m0{ secondDerivatives[k][j] };
            const T m1{ secondDerivatives[k + 1][j] };
            const T d{ (m1 - m0) * invH / T{ 6 } };
            const T beta{ (values[k + 1][j] - values[k][j]) * invH - h * (T{ 2 } * m0 + m1) / T{ 6 } };

            point.value[j] = values[k][j] + tau * (beta + tau * (m0 / T{ 2 } + tau * d));
            point.firstDerivative[j] = beta + tau * (m0 + T{ 3 } * d * tau);
            point.secondDerivative[j] = m0 + T{ 6 } * d * tau;
        }

        return point;
    }

    template<typename T, std::size_t Dimension, std::size_t MaxKnots>
    T CubicSpline<T, Dimension, MaxKnots>::First() const
    {
        really_assert(count >= 2);
        return knots[0];
    }

    template<typename T, std::size_t Dimension, std::size_t MaxKnots>
    T CubicSpline<T, Dimension, MaxKnots>::Last() const
    {
        really_assert(count >= 2);
        return knots[count - 1];
    }

    template<typename T, std::size_t Dimension, std::size_t MaxKnots>
    bool CubicSpline<T, Dimension, MaxKnots>::IsStrictlyIncreasing(const Knots& x, std::size_t n)
    {
        for (std::size_t k = 0; k + 1 < n; ++k)
            if (x[k + 1] - x[k] <= std::numeric_limits<T>::epsilon() * std::max(T{ 1 }, math::Abs(x[k])))
                return false;

        return true;
    }

    template<typename T, std::size_t Dimension, std::size_t MaxKnots>
    bool CubicSpline<T, Dimension, MaxKnots>::FactorSystem(TridiagonalSolver<T, MaxKnots>& solver, SplineEnd condition) const
    {
        Band sub{};
        Band diag{};
        Band sup{};
        const std::size_t last{ count - 1 };

        for (std::size_t k = 1; k < last; ++k)
        {
            const T before{ knots[k] - knots[k - 1] };
            const T after{ knots[k + 1] - knots[k] };
            sub[k] = before;
            diag[k] = T{ 2 } * (before + after);
            sup[k] = after;
        }

        if (condition == SplineEnd::Natural)
        {
            diag[0] = T{ 1 };
            diag[last] = T{ 1 };
        }
        else
        {
            const T firstSpan{ knots[1] - knots[0] };
            const T lastSpan{ knots[last] - knots[last - 1] };
            diag[0] = T{ 2 } * firstSpan;
            sup[0] = firstSpan;
            sub[last] = lastSpan;
            diag[last] = T{ 2 } * lastSpan;
        }

        return solver.Factor(sub, diag, sup, count);
    }

    template<typename T, std::size_t Dimension, std::size_t MaxKnots>
    auto CubicSpline<T, Dimension, MaxKnots>::RightHandSide(std::size_t channel, const SplineBoundary<T, Dimension>& boundary) const -> Band
    {
        Band rhs{};
        const std::size_t last{ count - 1 };
        const auto slope{ [this, channel](std::size_t k)
            {
                return (values[k + 1][channel] - values[k][channel]) * inverseSpan[k];
            } };

        for (std::size_t k = 1; k < last; ++k)
            rhs[k] = T{ 6 } * (slope(k) - slope(k - 1));

        if (boundary.condition == SplineEnd::Clamped)
        {
            rhs[0] = T{ 6 } * (slope(0) - boundary.startSlope[channel]);
            rhs[last] = T{ 6 } * (boundary.endSlope[channel] - slope(last - 1));
        }

        return rhs;
    }

    template<typename T, std::size_t Dimension, std::size_t MaxKnots>
    std::size_t CubicSpline<T, Dimension, MaxKnots>::Interval(T x) const
    {
        const auto upper{ std::upper_bound(knots.begin() + 1, knots.begin() + count, x) };
        const auto index{ static_cast<std::size_t>(upper - knots.begin()) - 1 };
        return std::min(index, count - 2);
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class CubicSpline<float, 3, 8>;
#endif
}
