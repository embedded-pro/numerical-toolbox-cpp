#include "numerical/math/Math.hpp"

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD

#ifndef MATH_ABS_OVERRIDE
template float math::Abs<float>(float);
#endif
#ifndef MATH_SQRT_OVERRIDE
template float math::Sqrt<float>(float);
#endif
#ifndef MATH_SIN_OVERRIDE
template float math::Sin<float>(float);
#endif
#ifndef MATH_COS_OVERRIDE
template float math::Cos<float>(float);
#endif
#ifndef MATH_TAN_OVERRIDE
template float math::Tan<float>(float);
#endif
#ifndef MATH_ASIN_OVERRIDE
template float math::Asin<float>(float);
#endif
#ifndef MATH_ACOS_OVERRIDE
template float math::Acos<float>(float);
#endif
#ifndef MATH_ATAN_OVERRIDE
template float math::Atan<float>(float);
#endif
#ifndef MATH_ATAN2_OVERRIDE
template float math::Atan2<float>(float, float);
#endif
#ifndef MATH_EXP_OVERRIDE
template float math::Exp<float>(float);
#endif
#ifndef MATH_LOG_OVERRIDE
template float math::Log<float>(float);
#endif
#ifndef MATH_LOG10_OVERRIDE
template float math::Log10<float>(float);
#endif
#ifndef MATH_LOG2_OVERRIDE
template float math::Log2<float>(float);
#endif
#ifndef MATH_POW_OVERRIDE
template float math::Pow<float>(float, float);
#endif
#ifndef MATH_SINH_OVERRIDE
template float math::Sinh<float>(float);
#endif
#ifndef MATH_COSH_OVERRIDE
template float math::Cosh<float>(float);
#endif
#ifndef MATH_TANH_OVERRIDE
template float math::Tanh<float>(float);
#endif
#ifndef MATH_HYPOT_OVERRIDE
template float math::Hypot<float>(float, float);
#endif
#ifndef MATH_COPYSIGN_OVERRIDE
template float math::Copysign<float>(float, float);
#endif
#ifndef MATH_FMOD_OVERRIDE
template float math::Fmod<float>(float, float);
#endif
#ifndef MATH_CEIL_OVERRIDE
template float math::Ceil<float>(float);
#endif
#ifndef MATH_FLOOR_OVERRIDE
template float math::Floor<float>(float);
#endif
#ifndef MATH_ROUND_OVERRIDE
template float math::Round<float>(float);
#endif
#ifndef MATH_ERFC_OVERRIDE
template float math::Erfc<float>(float);
#endif

#endif
