#pragma once

#include "numerical/math/Matrix.hpp"
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <limits>

namespace math::test
{
    template<typename T, std::size_t N>
    math::Vector<T, N> DefaultFiniteDifferenceSteps(const math::Vector<T, N>& x)
    {
        static_assert(std::is_floating_point_v<T>, "DefaultFiniteDifferenceSteps requires a floating-point type");
        const T cbrtEps = std::cbrt(std::numeric_limits<T>::epsilon());
        math::Vector<T, N> h;
        for (std::size_t i = 0; i < N; ++i)
            h.at(i, 0) = cbrtEps * std::max(T{ 1 }, std::abs(x.at(i, 0)));
        return h;
    }

    template<typename T, std::size_t N, typename Func>
    math::Vector<T, N> CentralDifferenceGradient(Func f, const math::Vector<T, N>& x, const math::Vector<T, N>& h)
    {
        static_assert(std::is_floating_point_v<T>, "CentralDifferenceGradient requires a floating-point type");
        math::Vector<T, N> gradient;
        for (std::size_t i = 0; i < N; ++i)
        {
            math::Vector<T, N> xPlus = x;
            math::Vector<T, N> xMinus = x;
            xPlus.at(i, 0) += h.at(i, 0);
            xMinus.at(i, 0) -= h.at(i, 0);
            gradient.at(i, 0) = (f(xPlus) - f(xMinus)) / (xPlus.at(i, 0) - xMinus.at(i, 0));
        }
        return gradient;
    }

    template<typename T, std::size_t N, typename Func>
    math::Vector<T, N> CentralDifferenceGradient(Func f, const math::Vector<T, N>& x, T h)
    {
        math::Vector<T, N> steps;
        for (std::size_t i = 0; i < N; ++i)
            steps.at(i, 0) = h;
        return CentralDifferenceGradient(f, x, steps);
    }

    template<typename T, std::size_t N, typename Func>
    math::Vector<T, N> CentralDifferenceGradient(Func f, const math::Vector<T, N>& x)
    {
        return CentralDifferenceGradient(f, x, DefaultFiniteDifferenceSteps(x));
    }

    template<typename T, std::size_t N, typename Func>
    void ExpectGradientNear(const math::Vector<T, N>& analytic, Func f, const math::Vector<T, N>& x, const math::Vector<T, N>& h, T tol)
    {
        const auto numeric = CentralDifferenceGradient(f, x, h);
        for (std::size_t i = 0; i < N; ++i)
        {
            const T a = analytic.at(i, 0);
            const T n = numeric.at(i, 0);
            const T absErr = std::abs(a - n);
            const T relErr = absErr / (std::max(std::abs(a), std::abs(n)) + std::numeric_limits<T>::min());
            if (absErr > tol && relErr > tol)
                ADD_FAILURE() << "component[" << i << "] analytic=" << a << " numeric=" << n
                              << " absErr=" << absErr << " relErr=" << relErr << " tol=" << tol;
        }
    }

    template<typename T, std::size_t N, typename Func>
    void ExpectGradientNear(const math::Vector<T, N>& analytic, Func f, const math::Vector<T, N>& x, T h, T tol)
    {
        math::Vector<T, N> steps;
        for (std::size_t i = 0; i < N; ++i)
            steps.at(i, 0) = h;
        ExpectGradientNear(analytic, f, x, steps, tol);
    }

    template<typename T, std::size_t N, typename Func>
    void ExpectGradientNear(const math::Vector<T, N>& analytic, Func f, const math::Vector<T, N>& x, T tol)
    {
        ExpectGradientNear(analytic, f, x, DefaultFiniteDifferenceSteps(x), tol);
    }
}
