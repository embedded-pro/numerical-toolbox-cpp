#include "numerical/math/LinearTimeInvariant.hpp"

namespace math
{
    template struct LinearTimeInvariant<float, 2, 1>;
    template struct LinearTimeInvariant<float, 2, 1, 1>;
    template struct LinearTimeInvariant<float, 3, 1>;
    template struct LinearTimeInvariant<float, 4, 1>;
    template struct LinearTimeInvariant<float, 4, 1, 1>;
    template struct LinearTimeInvariant<float, 2, 2>;
    template struct LinearTimeInvariant<math::Q15, 2, 1>;
    template struct LinearTimeInvariant<math::Q15, 2, 1, 1>;
    template struct LinearTimeInvariant<math::Q31, 2, 1>;
    template struct LinearTimeInvariant<math::Q31, 2, 1, 1>;
}
