#include "numerical/math/AngleUtilities.hpp"

namespace math
{
    template float WrapToPi<float>(float);
    template float WrapToTwoPi<float>(float);
    template float AngleDifference<float>(float, float);
    template float UnwrapAngle<float>(float, float);
}
