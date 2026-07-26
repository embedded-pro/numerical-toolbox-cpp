#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/analysis/FastFourierTransform.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/ComplexNumber.hpp"
#include <cstddef>
#include <infra/util/BoundedVector.hpp>
#include <type_traits>

namespace analysis
{
    template<typename T, std::size_t M, std::size_t K>
    OPTIMIZE_FOR_SPEED void LinearConvolution(
        const typename infra::BoundedVector<T>::template WithMaxSize<M>& x,
        const typename infra::BoundedVector<T>::template WithMaxSize<K>& h,
        typename infra::BoundedVector<T>::template WithMaxSize<M + K - 1>& y)
    {
        static_assert(std::is_floating_point_v<T>, "ConvolutionCorrelation supports floating-point types only");
        y.clear();
        y.resize(M + K - 1, T{ 0 });
        for (std::size_t n = 0; n < M + K - 1; ++n)
        {
            T acc{ 0 };
            std::size_t kStart{ n >= K - 1 ? n - (K - 1) : 0 };
            std::size_t kEnd{ n < M - 1 ? n : M - 1 };
            for (std::size_t k = kStart; k <= kEnd; ++k)
                acc += x[k] * h[n - k];
            y[n] = acc;
        }
    }

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED void CircularConvolution(
        const typename infra::BoundedVector<T>::template WithMaxSize<N>& x,
        const typename infra::BoundedVector<T>::template WithMaxSize<N>& h,
        typename infra::BoundedVector<T>::template WithMaxSize<N>& y)
    {
        static_assert(std::is_floating_point_v<T>, "ConvolutionCorrelation supports floating-point types only");
        y.clear();
        y.resize(N, T{ 0 });
        for (std::size_t n = 0; n < N; ++n)
        {
            T acc{ 0 };
            for (std::size_t k = 0; k < N; ++k)
                acc += x[k] * h[(n + N - k) % N];
            y[n] = acc;
        }
    }

    template<typename T, std::size_t M, std::size_t K>
    OPTIMIZE_FOR_SPEED void CrossCorrelation(
        const typename infra::BoundedVector<T>::template WithMaxSize<M>& x,
        const typename infra::BoundedVector<T>::template WithMaxSize<K>& y,
        typename infra::BoundedVector<T>::template WithMaxSize<M + K - 1>& r)
    {
        static_assert(std::is_floating_point_v<T>, "ConvolutionCorrelation supports floating-point types only");
        typename infra::BoundedVector<T>::template WithMaxSize<K> yRev;
        yRev.resize(K, T{ 0 });
        for (std::size_t i = 0; i < K; ++i)
            yRev[i] = y[K - 1 - i];
        LinearConvolution<T, M, K>(x, yRev, r);
    }

    template<typename T, std::size_t M>
    void AutoCorrelation(
        const typename infra::BoundedVector<T>::template WithMaxSize<M>& x,
        typename infra::BoundedVector<T>::template WithMaxSize<2 * M - 1>& r)
    {
        static_assert(std::is_floating_point_v<T>, "ConvolutionCorrelation supports floating-point types only");
        CrossCorrelation<T, M, M>(x, x, r);
    }

    template<typename T, std::size_t N>
    std::size_t ArgMaxLag(const typename infra::BoundedVector<T>::template WithMaxSize<N>& r)
    {
        static_assert(std::is_floating_point_v<T>, "ConvolutionCorrelation supports floating-point types only");
        std::size_t best{ 0 };
        for (std::size_t i = 1; i < N; ++i)
            if (r[i] > r[best])
                best = i;
        return best;
    }

    template<typename T, std::size_t M, std::size_t K, std::size_t L>
    void FastConvolution(
        const typename infra::BoundedVector<T>::template WithMaxSize<M>& x,
        const typename infra::BoundedVector<T>::template WithMaxSize<K>& h,
        typename infra::BoundedVector<T>::template WithMaxSize<M + K - 1>& y,
        FastFourierTransform<T>& fft)
    {
        static_assert(std::is_floating_point_v<T>, "ConvolutionCorrelation supports floating-point types only");
        static_assert((L & (L - 1)) == 0, "L must be a power of two");
        static_assert(L >= M + K - 1, "L must be at least M+K-1");

        typename infra::BoundedVector<T>::template WithMaxSize<L> xPad;
        typename infra::BoundedVector<T>::template WithMaxSize<L> hPad;
        xPad.resize(L, T{ 0 });
        hPad.resize(L, T{ 0 });

        for (std::size_t i = 0; i < M; ++i)
            xPad[i] = x[i];
        for (std::size_t i = 0; i < K; ++i)
            hPad[i] = h[i];

        typename infra::BoundedVector<math::Complex<T>>::template WithMaxSize<L> X;
        X.resize(L, math::Complex<T>{ T{ 0 }, T{ 0 } });
        {
            auto& rawX = fft.Forward(xPad);
            for (std::size_t i = 0; i < L; ++i)
                X[i] = rawX[i];
        }
        auto& H = fft.Forward(hPad);

        typename infra::BoundedVector<math::Complex<T>>::template WithMaxSize<L> Y;
        Y.resize(L, math::Complex<T>{ T{ 0 }, T{ 0 } });
        for (std::size_t i = 0; i < L; ++i)
            Y[i] = X[i] * H[i];

        auto& yFull = fft.Inverse(Y);

        y.clear();
        y.resize(M + K - 1, T{ 0 });
        for (std::size_t i = 0; i < M + K - 1; ++i)
            y[i] = yFull[i];
    }
}

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
namespace analysis
{
    extern template void LinearConvolution<float, 3, 3>(
        const infra::BoundedVector<float>::WithMaxSize<3>&,
        const infra::BoundedVector<float>::WithMaxSize<3>&,
        infra::BoundedVector<float>::WithMaxSize<5>&);

    extern template void CircularConvolution<float, 4>(
        const infra::BoundedVector<float>::WithMaxSize<4>&,
        const infra::BoundedVector<float>::WithMaxSize<4>&,
        infra::BoundedVector<float>::WithMaxSize<4>&);

    extern template void CrossCorrelation<float, 5, 5>(
        const infra::BoundedVector<float>::WithMaxSize<5>&,
        const infra::BoundedVector<float>::WithMaxSize<5>&,
        infra::BoundedVector<float>::WithMaxSize<9>&);

    extern template void AutoCorrelation<float, 4>(
        const infra::BoundedVector<float>::WithMaxSize<4>&,
        infra::BoundedVector<float>::WithMaxSize<7>&);

    extern template std::size_t ArgMaxLag<float, 9>(
        const infra::BoundedVector<float>::WithMaxSize<9>&);

    extern template void FastConvolution<float, 3, 3, 8>(
        const infra::BoundedVector<float>::WithMaxSize<3>&,
        const infra::BoundedVector<float>::WithMaxSize<3>&,
        infra::BoundedVector<float>::WithMaxSize<5>&,
        FastFourierTransform<float>&);
}
#endif
