#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include <bit>
#include <cmath>
#include <cstdint>
#include <type_traits>

#define MATH_ABS_OVERRIDE
#define MATH_SQRT_OVERRIDE
#define MATH_SIN_OVERRIDE
#define MATH_COS_OVERRIDE
#define MATH_TAN_OVERRIDE
#define MATH_ASIN_OVERRIDE
#define MATH_ACOS_OVERRIDE
#define MATH_ATAN_OVERRIDE
#define MATH_ATAN2_OVERRIDE
#define MATH_EXP_OVERRIDE
#define MATH_LOG_OVERRIDE
#define MATH_LOG10_OVERRIDE
#define MATH_LOG2_OVERRIDE
#define MATH_POW_OVERRIDE
#define MATH_SINH_OVERRIDE
#define MATH_COSH_OVERRIDE
#define MATH_TANH_OVERRIDE
#define MATH_HYPOT_OVERRIDE
#define MATH_COPYSIGN_OVERRIDE
#define MATH_FMOD_OVERRIDE
#define MATH_CEIL_OVERRIDE
#define MATH_FLOOR_OVERRIDE
#define MATH_ROUND_OVERRIDE
#define MATH_ERFC_OVERRIDE

#include "numerical/math/Math.hpp"

namespace math
{

#if defined(__ARM_ARCH_7EM__) && defined(__ARM_FP)

    // -------------------------------------------------------------------------
    // ARMv7E-M + FPv5-D16 (Cortex-M7) — double-precision FPU, FMA guaranteed
    // Detected: __ARM_FP bit 3 set means DP registers present
    // -------------------------------------------------------------------------
#if (__ARM_FP & 8) != 0

    template<typename T>
    T Abs(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_fabsf(x);
    }

    template<typename T>
    T Sqrt(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_sqrtf(x);
    }

    // VFMA.F32 + VSQRT.F32: single fused multiply-add eliminates intermediate rounding
    template<typename T>
    T Hypot(T x, T y)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_sqrtf(__builtin_fmaf(x, x, y * y));
    }

    template<typename T>
    T Copysign(T x, T y)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_copysignf(x, y);
    }

    template<typename T>
    T Fmod(T x, T y)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_fmodf(x, y);
    }

    template<typename T>
    T Ceil(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_ceilf(x);
    }

    template<typename T>
    T Floor(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_floorf(x);
    }

    template<typename T>
    T Round(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_roundf(x);
    }

    template<typename T>
    T Sin(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_sinf(x);
    }

    template<typename T>
    T Cos(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_cosf(x);
    }

    template<typename T>
    T Tan(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_tanf(x);
    }

    template<typename T>
    T Asin(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_asinf(x);
    }

    template<typename T>
    T Acos(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_acosf(x);
    }

    template<typename T>
    T Atan(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_atanf(x);
    }

    template<typename T>
    T Atan2(T y, T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_atan2f(y, x);
    }

    template<typename T>
    T Exp(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_expf(x);
    }

    template<typename T>
    T Log(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_logf(x);
    }

    template<typename T>
    T Log10(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_log10f(x);
    }

    template<typename T>
    T Log2(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_log2f(x);
    }

    template<typename T>
    T Pow(T base, T exp)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_powf(base, exp);
    }

    template<typename T>
    T Sinh(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_sinhf(x);
    }

    template<typename T>
    T Cosh(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_coshf(x);
    }

    template<typename T>
    T Tanh(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_tanhf(x);
    }

    template<typename T>
    T Erfc(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_erfcf(x);
    }

    // -------------------------------------------------------------------------
    // ARMv7E-M + FPv4-SP-D16 (Cortex-M4) — single-precision FPU only
    // Detected: __ARM_FP defined but no DP registers (__ARM_FP bit 3 clear)
    // -------------------------------------------------------------------------
#else

    template<typename T>
    T Abs(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_fabsf(x);
    }

    template<typename T>
    T Sqrt(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_sqrtf(x);
    }

    // No guaranteed VFMA on FPv4 pipeline — plain multiply+add + VSQRT.F32
    template<typename T>
    T Hypot(T x, T y)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_sqrtf(x * x + y * y);
    }

    template<typename T>
    T Copysign(T x, T y)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_copysignf(x, y);
    }

    template<typename T>
    T Fmod(T x, T y)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_fmodf(x, y);
    }

    template<typename T>
    T Ceil(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_ceilf(x);
    }

    template<typename T>
    T Floor(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_floorf(x);
    }

    template<typename T>
    T Round(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_roundf(x);
    }

    template<typename T>
    T Sin(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_sinf(x);
    }

    template<typename T>
    T Cos(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_cosf(x);
    }

    template<typename T>
    T Tan(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_tanf(x);
    }

    template<typename T>
    T Asin(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_asinf(x);
    }

    template<typename T>
    T Acos(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_acosf(x);
    }

    template<typename T>
    T Atan(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_atanf(x);
    }

    template<typename T>
    T Atan2(T y, T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_atan2f(y, x);
    }

    template<typename T>
    T Exp(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_expf(x);
    }

    template<typename T>
    T Log(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_logf(x);
    }

    template<typename T>
    T Log10(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_log10f(x);
    }

    template<typename T>
    T Log2(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_log2f(x);
    }

    template<typename T>
    T Pow(T base, T exp)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_powf(base, exp);
    }

    template<typename T>
    T Sinh(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_sinhf(x);
    }

    template<typename T>
    T Cosh(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_coshf(x);
    }

    template<typename T>
    T Tanh(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_tanhf(x);
    }

    template<typename T>
    T Erfc(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_erfcf(x);
    }

#endif // __ARM_FP DP check

#elif defined(__ARM_ARCH_8M_MAIN__) && defined(__ARM_FP)

    // -------------------------------------------------------------------------
    // ARMv8-M Mainline (Cortex-M33, M55) — FMA mandated by ISA when FPU present
    // __builtin_fmaf always maps to single VFMA.F32 on this core family
    // -------------------------------------------------------------------------

    template<typename T>
    T Abs(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_fabsf(x);
    }

    template<typename T>
    T Sqrt(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_sqrtf(x);
    }

    template<typename T>
    T Hypot(T x, T y)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_sqrtf(__builtin_fmaf(x, x, y * y));
    }

    template<typename T>
    T Copysign(T x, T y)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_copysignf(x, y);
    }

    template<typename T>
    T Fmod(T x, T y)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_fmodf(x, y);
    }

    template<typename T>
    T Ceil(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_ceilf(x);
    }

    template<typename T>
    T Floor(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_floorf(x);
    }

    template<typename T>
    T Round(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_roundf(x);
    }

    template<typename T>
    T Sin(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_sinf(x);
    }

    template<typename T>
    T Cos(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_cosf(x);
    }

    template<typename T>
    T Tan(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_tanf(x);
    }

    template<typename T>
    T Asin(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_asinf(x);
    }

    template<typename T>
    T Acos(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_acosf(x);
    }

    template<typename T>
    T Atan(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_atanf(x);
    }

    template<typename T>
    T Atan2(T y, T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_atan2f(y, x);
    }

    template<typename T>
    T Exp(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_expf(x);
    }

    template<typename T>
    T Log(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_logf(x);
    }

    template<typename T>
    T Log10(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_log10f(x);
    }

    template<typename T>
    T Log2(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_log2f(x);
    }

    template<typename T>
    T Pow(T base, T exp)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_powf(base, exp);
    }

    template<typename T>
    T Sinh(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_sinhf(x);
    }

    template<typename T>
    T Cosh(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_coshf(x);
    }

    template<typename T>
    T Tanh(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_tanhf(x);
    }

    template<typename T>
    T Erfc(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_erfcf(x);
    }

#elif defined(__ARM_ARCH_6M__) || defined(__ARM_ARCH_8M_BASE__)

    // -------------------------------------------------------------------------
    // ARMv6-M (Cortex-M0/M0+) and ARMv8-M Baseline (Cortex-M23) — no FPU
    // All float is soft-float ABI. Optimisations: integer bit-manipulation for
    // Abs/Copysign (no soft-float call), __builtin_*f forces float library path
    // and avoids accidental double promotion through std:: wrappers.
    // -------------------------------------------------------------------------

    // Bit-clear of sign bit: 2 integer Thumb instructions, no soft-float call
    template<typename T>
    T Abs(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        uint32_t bits = std::bit_cast<uint32_t>(x);
        bits &= 0x7FFFFFFFu;
        return std::bit_cast<T>(bits);
    }

    // Sign-bit transfer: 4 integer Thumb instructions, no soft-float call
    template<typename T>
    T Copysign(T x, T y)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        uint32_t xbits = std::bit_cast<uint32_t>(x) & 0x7FFFFFFFu;
        uint32_t ybits = std::bit_cast<uint32_t>(y) & 0x80000000u;
        return std::bit_cast<T>(xbits | ybits);
    }

    template<typename T>
    T Sqrt(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_sqrtf(x);
    }

    template<typename T>
    T Hypot(T x, T y)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_sqrtf(x * x + y * y);
    }

    template<typename T>
    T Fmod(T x, T y)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_fmodf(x, y);
    }

    template<typename T>
    T Ceil(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_ceilf(x);
    }

    template<typename T>
    T Floor(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_floorf(x);
    }

    template<typename T>
    T Round(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_roundf(x);
    }

    template<typename T>
    T Sin(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_sinf(x);
    }

    template<typename T>
    T Cos(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_cosf(x);
    }

    template<typename T>
    T Tan(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_tanf(x);
    }

    template<typename T>
    T Asin(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_asinf(x);
    }

    template<typename T>
    T Acos(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_acosf(x);
    }

    template<typename T>
    T Atan(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_atanf(x);
    }

    template<typename T>
    T Atan2(T y, T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_atan2f(y, x);
    }

    template<typename T>
    T Exp(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_expf(x);
    }

    template<typename T>
    T Log(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_logf(x);
    }

    template<typename T>
    T Log10(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_log10f(x);
    }

    template<typename T>
    T Log2(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_log2f(x);
    }

    template<typename T>
    T Pow(T base, T exp)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_powf(base, exp);
    }

    template<typename T>
    T Sinh(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_sinhf(x);
    }

    template<typename T>
    T Cosh(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_coshf(x);
    }

    template<typename T>
    T Tanh(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_tanhf(x);
    }

    template<typename T>
    T Erfc(T x)
    {
        static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");
        return __builtin_erfcf(x);
    }

#else
#error "MathArm.hpp included on an unsupported ARM architecture. Add a section for this core family."
#endif

}
