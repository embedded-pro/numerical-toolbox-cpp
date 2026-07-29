#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "infra/util/BoundedVector.hpp"
#include "numerical/analysis/FastFourierTransform.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/ComplexNumber.hpp"

namespace analysis
{
    template<typename T, std::size_t N>
    class RealFastFourierTransform
    {
        static_assert(std::is_floating_point_v<T>, "RealFastFourierTransform supports floating-point types");
        static_assert(N >= 4 && (N & (N - 1)) == 0, "RealFastFourierTransform length N must be an even power of two");

    public:
        using Complex = math::Complex<T>;
        using VectorComplex = infra::BoundedVector<Complex>;
        using VectorReal = infra::BoundedVector<T>;

        explicit RealFastFourierTransform(FastFourierTransform<T>& engine, TwiddleFactors<T, N / 2>& twiddleFactors);

        OPTIMIZE_FOR_SPEED VectorComplex& Forward(VectorReal& realInput);
        OPTIMIZE_FOR_SPEED VectorReal& Inverse(VectorComplex& halfSpectrum);

    private:
        static constexpr std::size_t half{ N / 2 };

        FastFourierTransform<T>& engine;
        TwiddleFactors<T, half>& twiddleFactors;
        typename VectorReal::template WithMaxSize<half> evenBuf;
        typename VectorReal::template WithMaxSize<half> oddBuf;
        typename VectorComplex::template WithMaxSize<half> eCopy;
        typename VectorComplex::template WithMaxSize<half + 1> spectrum;
        typename VectorReal::template WithMaxSize<N> timeDomain;
    };

    /// Implementation ///

    template<typename T, std::size_t N>
    RealFastFourierTransform<T, N>::RealFastFourierTransform(FastFourierTransform<T>& engine, TwiddleFactors<T, N / 2>& twiddleFactors)
        : engine{ engine }
        , twiddleFactors{ twiddleFactors }
    {
        evenBuf.resize(half);
        oddBuf.resize(half);
        eCopy.resize(half);
        spectrum.resize(half + 1);
        timeDomain.resize(N);
    }

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED typename RealFastFourierTransform<T, N>::VectorComplex& RealFastFourierTransform<T, N>::Forward(VectorReal& realInput)
    {
        for (std::size_t n{ 0 }; n < half; ++n)
        {
            evenBuf[n] = realInput[2 * n];
            oddBuf[n] = realInput[2 * n + 1];
        }

        VectorComplex& E{ engine.Forward(evenBuf) };
        for (std::size_t k{ 0 }; k < half; ++k)
            eCopy[k] = E[k];

        VectorComplex& O{ engine.Forward(oddBuf) };

        spectrum[0] = Complex{ eCopy[0].Real() + O[0].Real(), T(0) };
        spectrum[half] = Complex{ eCopy[0].Real() - O[0].Real(), T(0) };

        for (std::size_t k{ 1 }; k < half; ++k)
        {
            Complex twidO{ twiddleFactors[k] * O[k] };
            spectrum[k] = Complex{ eCopy[k].Real() + twidO.Real(), eCopy[k].Imaginary() + twidO.Imaginary() };
        }

        return spectrum;
    }

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED typename RealFastFourierTransform<T, N>::VectorReal& RealFastFourierTransform<T, N>::Inverse(VectorComplex& halfSpectrum)
    {
        typename VectorComplex::template WithMaxSize<half> E;
        typename VectorComplex::template WithMaxSize<half> O;
        E.resize(half);
        O.resize(half);

        E[0] = Complex{ T(0.5) * (halfSpectrum[0].Real() + halfSpectrum[half].Real()), T(0) };
        O[0] = Complex{ T(0.5) * (halfSpectrum[0].Real() - halfSpectrum[half].Real()), T(0) };

        for (std::size_t k{ 1 }; k < half; ++k)
        {
            Complex xk{ halfSpectrum[k] };
            Complex xNk{ halfSpectrum[half - k].Real(), -halfSpectrum[half - k].Imaginary() };
            Complex sum{ T(0.5) * (xk.Real() + xNk.Real()), T(0.5) * (xk.Imaginary() + xNk.Imaginary()) };
            Complex diff{ T(0.5) * (xk.Real() - xNk.Real()), T(0.5) * (xk.Imaginary() - xNk.Imaginary()) };
            Complex twConj{ twiddleFactors[k].Real(), -twiddleFactors[k].Imaginary() };
            E[k] = sum;
            O[k] = twConj * diff;
        }

        VectorReal& evenOut{ engine.Inverse(E) };
        typename VectorReal::template WithMaxSize<half> evenCopy;
        evenCopy.resize(half);
        for (std::size_t n{ 0 }; n < half; ++n)
            evenCopy[n] = evenOut[n];

        VectorReal& oddOut{ engine.Inverse(O) };

        for (std::size_t n{ 0 }; n < half; ++n)
        {
            timeDomain[2 * n] = evenCopy[n];
            timeDomain[2 * n + 1] = oddOut[n];
        }

        return timeDomain;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class RealFastFourierTransform<float, 16>;
#endif
}
