#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <type_traits>

namespace math
{
    template<typename T, std::size_t Iterations = 16>
    class Cordic
    {
        static_assert(std::is_floating_point_v<T>, "Cordic supports floating-point types only");

    public:
        struct SinCos
        {
            T sin;
            T cos;
        };

        struct PolarResult
        {
            T magnitude;
            T angle;
        };

        OPTIMIZE_FOR_SPEED SinCos SineCosine(T angleRadians) const;
        T Arctangent2(T y, T x) const;
        T Magnitude(T y, T x) const;
        std::array<T, 2> Rotate(std::array<T, 2> v, T angle) const;

    private:
        static constexpr std::array<T, Iterations> BuildAtanTable()
        {
            std::array<T, Iterations> table{};
            for (std::size_t i = 0; i < Iterations; ++i)
                table[i] = static_cast<T>(std::atan(std::pow(T(2), -static_cast<T>(i))));
            return table;
        }

        static constexpr T ComputeK()
        {
            T k{ T(1) };
            for (std::size_t i = 0; i < Iterations; ++i)
                k *= static_cast<T>(std::cos(std::atan(std::pow(T(2), -static_cast<T>(i)))));
            return k;
        }

        static constexpr std::array<T, Iterations> atanTable{ BuildAtanTable() };
        static constexpr T K{ ComputeK() };
    };

    template<typename T, std::size_t Iterations>
    OPTIMIZE_FOR_SPEED typename Cordic<T, Iterations>::SinCos Cordic<T, Iterations>::SineCosine(T angleRadians) const
    {
        const T pi{ std::numbers::pi_v<T> };
        const T halfPi{ pi / T(2) };

        T angle{ angleRadians };
        T sinSign{ T(1) };
        T cosSign{ T(1) };

        if (angle > halfPi)
        {
            angle -= pi;
            sinSign = T(-1);
            cosSign = T(-1);
        }
        else if (angle < -halfPi)
        {
            angle += pi;
            sinSign = T(-1);
            cosSign = T(-1);
        }

        T x{ K };
        T y{ T(0) };
        T z{ angle };

        for (std::size_t i = 0; i < Iterations; ++i)
        {
            T d{ (z >= T(0)) ? T(1) : T(-1) };
            T pow2i{ T(1) / static_cast<T>(std::size_t(1) << i) };
            T xNew{ x - d * y * pow2i };
            T yNew{ y + d * x * pow2i };
            z -= d * atanTable[i];
            x = xNew;
            y = yNew;
        }

        return SinCos{ sinSign * y, cosSign * x };
    }

    template<typename T, std::size_t Iterations>
    T Cordic<T, Iterations>::Arctangent2(T y, T x) const
    {
        const T pi{ std::numbers::pi_v<T> };

        T quadrantOffset{ T(0) };
        T xv{ x };
        T yv{ y };

        if (xv < T(0) && yv >= T(0))
        {
            xv = -x;
            yv = -y;
            quadrantOffset = pi;
        }
        else if (xv < T(0) && yv < T(0))
        {
            xv = -x;
            yv = -y;
            quadrantOffset = -pi;
        }

        T z{ T(0) };

        for (std::size_t i = 0; i < Iterations; ++i)
        {
            T d{ (yv >= T(0)) ? T(-1) : T(1) };
            T pow2i{ T(1) / static_cast<T>(std::size_t(1) << i) };
            T xNew{ xv - d * yv * pow2i };
            T yNew{ yv + d * xv * pow2i };
            z -= d * atanTable[i];
            xv = xNew;
            yv = yNew;
        }

        return z + quadrantOffset;
    }

    template<typename T, std::size_t Iterations>
    T Cordic<T, Iterations>::Magnitude(T y, T x) const
    {
        T xv{ (x < T(0)) ? -x : x };
        T yv{ (y < T(0)) ? -y : y };
        T z{ T(0) };

        for (std::size_t i = 0; i < Iterations; ++i)
        {
            T d{ (yv >= T(0)) ? T(-1) : T(1) };
            T pow2i{ T(1) / static_cast<T>(std::size_t(1) << i) };
            T xNew{ xv - d * yv * pow2i };
            T yNew{ yv + d * xv * pow2i };
            z -= d * atanTable[i];
            xv = xNew;
            yv = yNew;
        }

        return K * xv;
    }

    template<typename T, std::size_t Iterations>
    std::array<T, 2> Cordic<T, Iterations>::Rotate(std::array<T, 2> v, T angle) const
    {
        auto sc = SineCosine(angle);
        return std::array<T, 2>{ v[0] * sc.cos - v[1] * sc.sin, v[0] * sc.sin + v[1] * sc.cos };
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class Cordic<float, 16>;
    extern template class Cordic<float, 8>;
#endif
}
