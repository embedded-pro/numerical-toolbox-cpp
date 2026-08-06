#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include <cmath>
#include <type_traits>

namespace math
{
#ifndef MATH_ABS_OVERRIDE
    template<typename T>
    constexpr T Abs(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return std::abs(x);
    }
#endif

#ifndef MATH_SQRT_OVERRIDE
    template<typename T>
    constexpr T Sqrt(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return std::sqrtf(x);
    }
#endif

#ifndef MATH_SIN_OVERRIDE
    template<typename T>
    constexpr T Sin(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return std::sin(x);
    }
#endif

#ifndef MATH_COS_OVERRIDE
    template<typename T>
    constexpr T Cos(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return std::cos(x);
    }
#endif

#ifndef MATH_TAN_OVERRIDE
    template<typename T>
    constexpr T Tan(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return std::tan(x);
    }
#endif

#ifndef MATH_ASIN_OVERRIDE
    template<typename T>
    constexpr T Asin(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return std::asin(x);
    }
#endif

#ifndef MATH_ACOS_OVERRIDE
    template<typename T>
    constexpr T Acos(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return std::acos(x);
    }
#endif

#ifndef MATH_ATAN_OVERRIDE
    template<typename T>
    constexpr T Atan(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return std::atan(x);
    }
#endif

#ifndef MATH_ATAN2_OVERRIDE
    template<typename T>
    constexpr T Atan2(T y, T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return std::atan2(y, x);
    }
#endif

#ifndef MATH_EXP_OVERRIDE
    template<typename T>
    constexpr T Exp(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return std::exp(x);
    }
#endif

#ifndef MATH_LOG_OVERRIDE
    template<typename T>
    constexpr T Log(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return std::log(x);
    }
#endif

#ifndef MATH_LOG10_OVERRIDE
    template<typename T>
    constexpr T Log10(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return std::log10(x);
    }
#endif

#ifndef MATH_LOG2_OVERRIDE
    template<typename T>
    constexpr T Log2(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return std::log2(x);
    }
#endif

#ifndef MATH_POW_OVERRIDE
    template<typename T>
    constexpr T Pow(T base, T exp)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return std::pow(base, exp);
    }
#endif

#ifndef MATH_SINH_OVERRIDE
    template<typename T>
    constexpr T Sinh(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return std::sinh(x);
    }
#endif

#ifndef MATH_COSH_OVERRIDE
    template<typename T>
    constexpr T Cosh(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return std::cosh(x);
    }
#endif

#ifndef MATH_TANH_OVERRIDE
    template<typename T>
    constexpr T Tanh(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return std::tanh(x);
    }
#endif

#ifndef MATH_HYPOT_OVERRIDE
    template<typename T>
    constexpr T Hypot(T x, T y)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return std::hypot(x, y);
    }
#endif

#ifndef MATH_COPYSIGN_OVERRIDE
    template<typename T>
    constexpr T Copysign(T x, T y)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return std::copysign(x, y);
    }
#endif

#ifndef MATH_FMOD_OVERRIDE
    template<typename T>
    constexpr T Fmod(T x, T y)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return std::fmod(x, y);
    }
#endif

#ifndef MATH_CEIL_OVERRIDE
    template<typename T>
    constexpr T Ceil(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return std::ceil(x);
    }
#endif

#ifndef MATH_FLOOR_OVERRIDE
    template<typename T>
    constexpr T Floor(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return std::floor(x);
    }
#endif

#ifndef MATH_ROUND_OVERRIDE
    template<typename T>
    constexpr T Round(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return std::round(x);
    }
#endif

#ifndef MATH_ERFC_OVERRIDE
    template<typename T>
    constexpr T Erfc(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return std::erfc(x);
    }
#endif
}
