#include "numerical/math/Math.hpp"

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD

namespace math
{
#ifndef MATH_ABS_OVERRIDE
    template float Abs<float>(float);
#endif
#ifndef MATH_SQRT_OVERRIDE
    template float Sqrt<float>(float);
#endif
#ifndef MATH_SIN_OVERRIDE
    template float Sin<float>(float);
#endif
#ifndef MATH_COS_OVERRIDE
    template float Cos<float>(float);
#endif
#ifndef MATH_TAN_OVERRIDE
    template float Tan<float>(float);
#endif
#ifndef MATH_ASIN_OVERRIDE
    template float Asin<float>(float);
#endif
#ifndef MATH_ACOS_OVERRIDE
    template float Acos<float>(float);
#endif
#ifndef MATH_ATAN_OVERRIDE
    template float Atan<float>(float);
#endif
#ifndef MATH_ATAN2_OVERRIDE
    template float Atan2<float>(float, float);
#endif
#ifndef MATH_EXP_OVERRIDE
    template float Exp<float>(float);
#endif
#ifndef MATH_LOG_OVERRIDE
    template float Log<float>(float);
#endif
#ifndef MATH_LOG10_OVERRIDE
    template float Log10<float>(float);
#endif
#ifndef MATH_LOG2_OVERRIDE
    template float Log2<float>(float);
#endif
#ifndef MATH_POW_OVERRIDE
    template float Pow<float>(float, float);
#endif
#ifndef MATH_SINH_OVERRIDE
    template float Sinh<float>(float);
#endif
#ifndef MATH_COSH_OVERRIDE
    template float Cosh<float>(float);
#endif
#ifndef MATH_TANH_OVERRIDE
    template float Tanh<float>(float);
#endif
#ifndef MATH_HYPOT_OVERRIDE
    template float Hypot<float>(float, float);
#endif
#ifndef MATH_COPYSIGN_OVERRIDE
    template float Copysign<float>(float, float);
#endif
#ifndef MATH_FMOD_OVERRIDE
    template float Fmod<float>(float, float);
#endif
#ifndef MATH_CEIL_OVERRIDE
    template float Ceil<float>(float);
#endif
#ifndef MATH_FLOOR_OVERRIDE
    template float Floor<float>(float);
#endif
#ifndef MATH_ROUND_OVERRIDE
    template float Round<float>(float);
#endif
#ifndef MATH_ERFC_OVERRIDE
    template float Erfc<float>(float);
#endif
}

#endif
