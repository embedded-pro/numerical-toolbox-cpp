#include "numerical/controllers/implementations/SaturationRateLimiter.hpp"

namespace controllers
{
    template class Saturation<float>;
    template class RateLimiter<float>;
    template class SlewLimitedSaturation<float>;
}
