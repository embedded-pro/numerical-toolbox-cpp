#include "numerical/analysis/ConvolutionCorrelation.hpp"

namespace analysis
{
    template void LinearConvolution<float, 3, 3>(
        const infra::BoundedVector<float>::WithMaxSize<3>&,
        const infra::BoundedVector<float>::WithMaxSize<3>&,
        infra::BoundedVector<float>::WithMaxSize<5>&);

    template void CircularConvolution<float, 4>(
        const infra::BoundedVector<float>::WithMaxSize<4>&,
        const infra::BoundedVector<float>::WithMaxSize<4>&,
        infra::BoundedVector<float>::WithMaxSize<4>&);

    template void CrossCorrelation<float, 5, 5>(
        const infra::BoundedVector<float>::WithMaxSize<5>&,
        const infra::BoundedVector<float>::WithMaxSize<5>&,
        infra::BoundedVector<float>::WithMaxSize<9>&);

    template void AutoCorrelation<float, 4>(
        const infra::BoundedVector<float>::WithMaxSize<4>&,
        infra::BoundedVector<float>::WithMaxSize<7>&);

    template std::size_t ArgMaxLag<float, 9>(
        const infra::BoundedVector<float>::WithMaxSize<9>&);
}
