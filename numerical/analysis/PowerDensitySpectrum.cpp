#include "numerical/analysis/PowerDensitySpectrum.hpp"
#include "numerical/analysis/test/PowerDensitySpectrumTestSupport.hpp"

namespace analysis
{
    template class PowerSpectralDensity<float, 512, test::FftStub<float, 512>, test::TwiddleFactorsStub<float, 256>, 50>;
}
