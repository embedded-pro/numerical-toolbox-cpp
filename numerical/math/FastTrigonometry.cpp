#include "numerical/math/FastTrigonometry.hpp"

namespace math
{
    template float FastSine<float>(float);
    template float FastCosine<float>(float);
    template SineAndCosine<float> AccurateSineAndCosine<float>(float);
}
