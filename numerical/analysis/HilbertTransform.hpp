#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "infra/util/BoundedVector.hpp"
#include "numerical/analysis/FastFourierTransform.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/ComplexNumber.hpp"
#include "numerical/math/Math.hpp"
#include <array>
#include <numbers>
#include <type_traits>

namespace analysis
{
    template<typename T, std::size_t N>
    class AnalyticSignalFft
    {
        static_assert(std::is_floating_point_v<T>, "AnalyticSignalFft supports floating-point types");
        static_assert(N >= 4 && (N & (N - 1)) == 0, "AnalyticSignalFft length N must be a power of two >= 4");

    public:
        using Complex = math::Complex<T>;
        using VectorComplex = infra::BoundedVector<Complex>;
        using VectorReal = infra::BoundedVector<T>;

        explicit AnalyticSignalFft(FastFourierTransform<T>& fft);

        OPTIMIZE_FOR_SPEED VectorComplex& Analytic(VectorReal& x);

        static T InstantaneousAmplitude(Complex a);
        static T InstantaneousPhase(Complex a);
        static T InstantaneousFrequency(T phaseNow, T phasePrev, T ts);

    private:
        FastFourierTransform<T>& fft;
        typename VectorComplex::template WithMaxSize<N> spectrum;
        typename VectorComplex::template WithMaxSize<N> analytic;
    };

    template<typename T, std::size_t Taps>
    class HilbertFir
    {
        static_assert(std::is_floating_point_v<T>, "HilbertFir supports floating-point types");
        static_assert(Taps >= 3 && (Taps % 2) == 1, "HilbertFir requires an odd number of taps >= 3");

    public:
        using Complex = math::Complex<T>;

        HilbertFir();

        OPTIMIZE_FOR_SPEED Complex Filter(T x);

    private:
        static constexpr std::size_t centerTap{ (Taps - 1) / 2 };

        std::array<T, Taps> coeff{};
        std::array<T, Taps> delay{};
        std::size_t writeIndex{ 0 };
    };

    /// AnalyticSignalFft implementation ///

    template<typename T, std::size_t N>
    AnalyticSignalFft<T, N>::AnalyticSignalFft(FastFourierTransform<T>& fft)
        : fft{ fft }
    {
        spectrum.resize(N);
        analytic.resize(N);
    }

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED typename AnalyticSignalFft<T, N>::VectorComplex& AnalyticSignalFft<T, N>::Analytic(VectorReal& x)
    {
        VectorComplex& X{ fft.Forward(x) };

        spectrum[0] = X[0];
        spectrum[N / 2] = X[N / 2];

        for (std::size_t k{ 1 }; k < N / 2; ++k)
            spectrum[k] = Complex{ T(2) * X[k].Real(), T(2) * X[k].Imaginary() };

        for (std::size_t k{ N / 2 + 1 }; k < N; ++k)
            spectrum[k] = Complex{ T(0), T(0) };

        typename VectorComplex::template WithMaxSize<N> hilbertSpectrum{};
        hilbertSpectrum.resize(N);
        for (std::size_t k{ 0 }; k < N; ++k)
            hilbertSpectrum[k] = Complex{ spectrum[k].Imaginary(), -spectrum[k].Real() };

        VectorReal& hilbertTd{ fft.Inverse(hilbertSpectrum) };

        for (std::size_t n{ 0 }; n < N; ++n)
            analytic[n] = Complex{ x[n], hilbertTd[n] };

        return analytic;
    }

    template<typename T, std::size_t N>
    T AnalyticSignalFft<T, N>::InstantaneousAmplitude(Complex a)
    {
        return math::Sqrt(a.Real() * a.Real() + a.Imaginary() * a.Imaginary());
    }

    template<typename T, std::size_t N>
    T AnalyticSignalFft<T, N>::InstantaneousPhase(Complex a)
    {
        return math::Atan2(a.Imaginary(), a.Real());
    }

    template<typename T, std::size_t N>
    T AnalyticSignalFft<T, N>::InstantaneousFrequency(T phaseNow, T phasePrev, T ts)
    {
        T dphi{ phaseNow - phasePrev };
        while (dphi > std::numbers::pi_v<T>)
            dphi -= T(2) * std::numbers::pi_v<T>;
        while (dphi < -std::numbers::pi_v<T>)
            dphi += T(2) * std::numbers::pi_v<T>;
        return dphi / (T(2) * std::numbers::pi_v<T> * ts);
    }

    /// HilbertFir implementation ///

    template<typename T, std::size_t Taps>
    HilbertFir<T, Taps>::HilbertFir()
    {
        coeff.fill(T(0));
        delay.fill(T(0));
        for (std::size_t i{ 0 }; i < Taps; ++i)
        {
            int k{ static_cast<int>(i) - static_cast<int>(centerTap) };
            if (k != 0 && (k % 2) != 0)
            {
                T w{ T(0.54) - T(0.46) * math::Cos(T(2) * std::numbers::pi_v<T> * static_cast<T>(i) / static_cast<T>(Taps - 1)) };
                coeff[i] = (T(2) / (std::numbers::pi_v<T> * static_cast<T>(k))) * w;
            }
        }
    }

    template<typename T, std::size_t Taps>
    OPTIMIZE_FOR_SPEED typename HilbertFir<T, Taps>::Complex HilbertFir<T, Taps>::Filter(T x)
    {
        delay[writeIndex] = x;

        T imag{ T(0) };
        for (std::size_t i{ 0 }; i < Taps; ++i)
        {
            std::size_t idx{ (writeIndex + Taps - i) % Taps };
            imag += coeff[i] * delay[idx];
        }

        std::size_t realIdx{ (writeIndex + Taps - centerTap) % Taps };
        T real{ delay[realIdx] };

        writeIndex = (writeIndex + 1) % Taps;

        return Complex{ real, imag };
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class AnalyticSignalFft<float, 64>;
    extern template class HilbertFir<float, 31>;
#endif
}
