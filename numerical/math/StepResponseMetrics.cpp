#include "numerical/math/StepResponseMetrics.hpp"

namespace math
{
    template float RiseTime<float, 64>(const Vector<float, 64>&, float, float);
    template float SettlingTime<float, 64>(const Vector<float, 64>&, float, float, float);
    template float PercentOvershoot<float, 64>(const Vector<float, 64>&, float);
    template float PeakTime<float, 64>(const Vector<float, 64>&, float);
    template float SteadyStateError<float, 64>(const Vector<float, 64>&, float);
}
