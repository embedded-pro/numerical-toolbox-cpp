#include "numerical/math/Math.hpp"

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD

#ifndef MATH_ABS_OVERRIDE
namespace math { template float Abs<float>(float); }
#endif
#ifndef MATH_SQRT_OVERRIDE
namespace math { template float Sqrt<float>(float); }
#endif
#ifndef MATH_SIN_OVERRIDE
namespace math { template float Sin<float>(float); }
#endif
#ifndef MATH_COS_OVERRIDE
namespace math { template float Cos<float>(float); }
#endif
#ifndef MATH_TAN_OVERRIDE
namespace math { template float Tan<float>(float); }
#endif
#ifndef MATH_ASIN_OVERRIDE
namespace math { template float Asin<float>(float); }
#endif
#ifndef MATH_ACOS_OVERRIDE
namespace math { template float Acos<float>(float); }
#endif
#ifndef MATH_ATAN_OVERRIDE
namespace math { template float Atan<float>(float); }
#endif
#ifndef MATH_ATAN2_OVERRIDE
namespace math { template float Atan2<float>(float, float); }
#endif
#ifndef MATH_EXP_OVERRIDE
namespace math { template float Exp<float>(float); }
#endif
#ifndef MATH_LOG_OVERRIDE
namespace math { template float Log<float>(float); }
#endif
#ifndef MATH_LOG10_OVERRIDE
namespace math { template float Log10<float>(float); }
#endif
#ifndef MATH_LOG2_OVERRIDE
namespace math { template float Log2<float>(float); }
#endif
#ifndef MATH_POW_OVERRIDE
namespace math { template float Pow<float>(float, float); }
#endif
#ifndef MATH_SINH_OVERRIDE
namespace math { template float Sinh<float>(float); }
#endif
#ifndef MATH_COSH_OVERRIDE
namespace math { template float Cosh<float>(float); }
#endif
#ifndef MATH_TANH_OVERRIDE
namespace math { template float Tanh<float>(float); }
#endif
#ifndef MATH_HYPOT_OVERRIDE
namespace math { template float Hypot<float>(float, float); }
#endif
#ifndef MATH_COPYSIGN_OVERRIDE
namespace math { template float Copysign<float>(float, float); }
#endif
#ifndef MATH_FMOD_OVERRIDE
namespace math { template float Fmod<float>(float, float); }
#endif
#ifndef MATH_CEIL_OVERRIDE
namespace math { template float Ceil<float>(float); }
#endif
#ifndef MATH_FLOOR_OVERRIDE
namespace math { template float Floor<float>(float); }
#endif
#ifndef MATH_ROUND_OVERRIDE
namespace math { template float Round<float>(float); }
#endif
#ifndef MATH_ERFC_OVERRIDE
namespace math { template float Erfc<float>(float); }
#endif

#endif
