#include "numerical/filters/active/ParticleFilter.hpp"

namespace filters
{
    template class ParticleFilter<float, 3, 64>;
    template class ParticleFilter<float, 1, 41>;
    template class ParticleFilter<float, 1, 4>;
    template void ParticleFilter<float, 1, 41>::Initialize<math::Vector<float, 1> (&)(std::size_t)>(math::Vector<float, 1> (&)(std::size_t));
    template void ParticleFilter<float, 1, 4>::Initialize<math::Vector<float, 1> (&)(std::size_t)>(math::Vector<float, 1> (&)(std::size_t));
    template bool ParticleFilter<float, 1, 41>::Update<float (&)(const math::Vector<float, 1>&)>(float (&)(const math::Vector<float, 1>&));
    template bool ParticleFilter<float, 1, 4>::Update<float (&)(const math::Vector<float, 1>&)>(float (&)(const math::Vector<float, 1>&));
    template void ParticleFilter<float, 1, 4>::Predict<void (&)(math::Vector<float, 1>&, std::size_t)>(void (&)(math::Vector<float, 1>&, std::size_t));
}
