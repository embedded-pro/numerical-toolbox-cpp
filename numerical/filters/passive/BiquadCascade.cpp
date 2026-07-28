#include "numerical/filters/passive/BiquadCascade.hpp"

namespace filters::passive
{
    template struct BiquadCoeffs<float>;
    template class Biquad<float>;
    template class BiquadCascade<float, 2>;
}
