#include "numerical/analysis/ConvolutionCorrelation.hpp"

template void analysis::LinearConvolution<float, 3, 3>(
    const infra::BoundedVector<float>::WithMaxSize<3>&,
    const infra::BoundedVector<float>::WithMaxSize<3>&,
    infra::BoundedVector<float>::WithMaxSize<5>&);

template void analysis::CircularConvolution<float, 4>(
    const infra::BoundedVector<float>::WithMaxSize<4>&,
    const infra::BoundedVector<float>::WithMaxSize<4>&,
    infra::BoundedVector<float>::WithMaxSize<4>&);

template void analysis::CrossCorrelation<float, 5, 5>(
    const infra::BoundedVector<float>::WithMaxSize<5>&,
    const infra::BoundedVector<float>::WithMaxSize<5>&,
    infra::BoundedVector<float>::WithMaxSize<9>&);

template void analysis::AutoCorrelation<float, 4>(
    const infra::BoundedVector<float>::WithMaxSize<4>&,
    infra::BoundedVector<float>::WithMaxSize<7>&);

template std::size_t analysis::ArgMaxLag<float, 9>(
    const infra::BoundedVector<float>::WithMaxSize<9>&);

template void analysis::FastConvolution<float, 3, 3, 8>(
    const infra::BoundedVector<float>::WithMaxSize<3>&,
    const infra::BoundedVector<float>::WithMaxSize<3>&,
    infra::BoundedVector<float>::WithMaxSize<5>&,
    analysis::FastFourierTransform<float>&);
