#pragma once

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize("O3", "fast-math")
#endif

#include "infra/util/BoundedVector.hpp"
#include "infra/util/ReallyAssert.hpp"
#include "numerical/analysis/MelFilterbank.hpp"
#include "numerical/analysis/RealFastFourierTransform.hpp"
#include "numerical/analysis/windowing/Windowing.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <numbers>

namespace analysis
{
    template<typename T, std::size_t FftSize, std::size_t NumMelBands, std::size_t NumCoefficients>
    class Mfcc
    {
        static_assert(std::is_floating_point_v<T>, "Mfcc supports floating-point types");
        static_assert(NumCoefficients >= 1 && NumCoefficients <= NumMelBands, "Mfcc NumCoefficients must be in [1, NumMelBands]");

    public:
        static constexpr std::size_t NumBins{ FftSize / 2 + 1 };

        using VectorReal = infra::BoundedVector<T>;

        Mfcc(windowing::Window<T>& window, RealFastFourierTransform<T, FftSize>& rfft, const MelFilterbank<T, FftSize, NumMelBands>& filterbank, T logFloor);

        OPTIMIZE_FOR_SPEED const VectorReal& Compute(const VectorReal& frame);
        const VectorReal& LogMelEnergies() const;

    private:
        void ComputeLogMel(const VectorReal& frame);
        void ComputeCepstrum();

        RealFastFourierTransform<T, FftSize>& rfft;
        const MelFilterbank<T, FftSize, NumMelBands>& filterbank;
        T logFloor;

        std::array<T, FftSize> windowCoefficients{};
        std::array<std::array<T, NumMelBands>, NumCoefficients> dctBasis{};

        typename VectorReal::template WithMaxSize<FftSize> windowed;
        typename VectorReal::template WithMaxSize<NumBins> power;
        typename VectorReal::template WithMaxSize<NumMelBands> logMel;
        typename VectorReal::template WithMaxSize<NumCoefficients> coefficients;
    };

    template<typename T, std::size_t FftSize, std::size_t NumMelBands, std::size_t NumCoefficients>
    Mfcc<T, FftSize, NumMelBands, NumCoefficients>::Mfcc(windowing::Window<T>& window, RealFastFourierTransform<T, FftSize>& rfft, const MelFilterbank<T, FftSize, NumMelBands>& filterbank, T logFloor)
        : rfft{ rfft }
        , filterbank{ filterbank }
        , logFloor{ logFloor }
    {
        really_assert(logFloor > T(0));

        for (std::size_t n = 0; n < FftSize; ++n)
            windowCoefficients[n] = window(n, FftSize);

        const T bands{ static_cast<T>(NumMelBands) };
        for (std::size_t k = 0; k < NumCoefficients; ++k)
        {
            const T scale{ math::Sqrt((k == 0 ? T(1) : T(2)) / bands) };
            for (std::size_t n = 0; n < NumMelBands; ++n)
                dctBasis[k][n] = scale * math::Cos(std::numbers::pi_v<T> * static_cast<T>(k) * (T(2) * static_cast<T>(n) + T(1)) / (T(2) * bands));
        }

        windowed.resize(FftSize);
        power.resize(NumBins);
        logMel.resize(NumMelBands);
        coefficients.resize(NumCoefficients);
    }

    template<typename T, std::size_t FftSize, std::size_t NumMelBands, std::size_t NumCoefficients>
    OPTIMIZE_FOR_SPEED const typename Mfcc<T, FftSize, NumMelBands, NumCoefficients>::VectorReal& Mfcc<T, FftSize, NumMelBands, NumCoefficients>::Compute(const VectorReal& frame)
    {
        really_assert(frame.size() == FftSize);

        ComputeLogMel(frame);
        ComputeCepstrum();
        return coefficients;
    }

    template<typename T, std::size_t FftSize, std::size_t NumMelBands, std::size_t NumCoefficients>
    const typename Mfcc<T, FftSize, NumMelBands, NumCoefficients>::VectorReal& Mfcc<T, FftSize, NumMelBands, NumCoefficients>::LogMelEnergies() const
    {
        return logMel;
    }

    template<typename T, std::size_t FftSize, std::size_t NumMelBands, std::size_t NumCoefficients>
    void Mfcc<T, FftSize, NumMelBands, NumCoefficients>::ComputeLogMel(const VectorReal& frame)
    {
        for (std::size_t n = 0; n < FftSize; ++n)
            windowed[n] = frame[n] * windowCoefficients[n];

        const auto& spectrum{ rfft.Forward(windowed) };
        for (std::size_t k = 0; k < NumBins; ++k)
            power[k] = spectrum[k].Real() * spectrum[k].Real() + spectrum[k].Imaginary() * spectrum[k].Imaginary();

        filterbank.Apply(power, logMel);

        for (std::size_t m = 0; m < NumMelBands; ++m)
            logMel[m] = math::Log(std::max(logMel[m], logFloor));
    }

    template<typename T, std::size_t FftSize, std::size_t NumMelBands, std::size_t NumCoefficients>
    void Mfcc<T, FftSize, NumMelBands, NumCoefficients>::ComputeCepstrum()
    {
        for (std::size_t k = 0; k < NumCoefficients; ++k)
        {
            T sum{ T(0) };
            for (std::size_t n = 0; n < NumMelBands; ++n)
                sum += dctBasis[k][n] * logMel[n];
            coefficients[k] = sum;
        }
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class Mfcc<float, 256, 20, 13>;
#endif
}

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif
