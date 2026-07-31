#include "numerical/filters/active/AhrsMadgwickMahony.hpp"

namespace filters
{
    template class AhrsFilter<float, AhrsMode::Madgwick>;
    template class AhrsFilter<float, AhrsMode::Mahony>;
}
