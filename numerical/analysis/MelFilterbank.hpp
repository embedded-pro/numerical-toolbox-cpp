#pragma once

#include "infra/util/BoundedVector.hpp"
#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace analysis
{
    enum class MelScale : uint8_t
    {
        Htk,
        Slaney
    };

    enum class MelNormalization : uint8_t
    {
        None,
        Slaney
    };

    template<typename T>
    T HzToMel(T hz, MelScale scale)
    {
        static_assert(std::is_floating_point_v<T>, "HzToMel supports floating-point types");

        if (scale == MelScale::Htk)
            return T(2595) * math::Log10(T(1) + hz / T(700));

        constexpr T linearSlope{ T(200) / T(3) };
        constexpr T breakHz{ T(1000) };
        constexpr T breakMel{ breakHz / linearSlope };

        if (hz < breakHz)
            return hz / linearSlope;

        return breakMel + math::Log(hz / breakHz) * T(27) / math::Log(T(6.4));
    }

    template<typename T>
    T MelToHz(T mel, MelScale scale)
    {
        static_assert(std::is_floating_point_v<T>, "MelToHz supports floating-point types");

        if (scale == MelScale::Htk)
            return T(700) * (math::Pow(T(10), mel / T(2595)) - T(1));

        constexpr T linearSlope{ T(200) / T(3) };
        constexpr T breakHz{ T(1000) };
        constexpr T breakMel{ breakHz / linearSlope };

        if (mel < breakMel)
            return mel * linearSlope;

        return breakHz * math::Exp((mel - breakMel) * math::Log(T(6.4)) / T(27));
    }

    template<typename T, std::size_t FftSize, std::size_t NumMelBands>
    class MelFilterbank
    {
        static_assert(std::is_floating_point_v<T>, "MelFilterbank supports floating-point types");
        static_assert(FftSize >= 4 && (FftSize & (FftSize - 1)) == 0, "MelFilterbank FftSize must be a power of two >= 4");
        static_assert(NumMelBands >= 1, "MelFilterbank needs at least one band");
        static_assert(NumMelBands < std::numeric_limits<uint16_t>::max(), "MelFilterbank NumMelBands too large");

    public:
        static constexpr std::size_t NumBins{ FftSize / 2 + 1 };

        MelFilterbank(T sampleRate, T fMin, T fMax, MelScale scale, MelNormalization normalization);

        OPTIMIZE_FOR_SPEED void Apply(const infra::BoundedVector<T>& powerSpectrum, infra::BoundedVector<T>& melEnergies) const;
        T Weight(std::size_t band, std::size_t bin) const;

    private:
        static constexpr uint16_t outsideBands{ std::numeric_limits<uint16_t>::max() };

        void AssignBinsToSegments(T sampleRate, const std::array<T, NumMelBands + 2>& edgesHz);

        std::array<uint16_t, NumBins> segment{};
        std::array<T, NumBins> risingWeight{};
        std::array<T, NumMelBands> bandGain{};
    };

    template<typename T, std::size_t FftSize, std::size_t NumMelBands>
    MelFilterbank<T, FftSize, NumMelBands>::MelFilterbank(T sampleRate, T fMin, T fMax, MelScale scale, MelNormalization normalization)
    {
        really_assert(sampleRate > T(0));
        really_assert(fMin >= T(0) && fMin < fMax && fMax <= sampleRate / T(2));

        const T melLow{ HzToMel(fMin, scale) };
        const T melStep{ (HzToMel(fMax, scale) - melLow) / static_cast<T>(NumMelBands + 1) };

        std::array<T, NumMelBands + 2> edgesHz{};
        for (std::size_t i = 0; i < edgesHz.size(); ++i)
            edgesHz[i] = MelToHz(melLow + static_cast<T>(i) * melStep, scale);

        for (std::size_t m = 0; m < NumMelBands; ++m)
            bandGain[m] = normalization == MelNormalization::Slaney ? T(2) / (edgesHz[m + 2] - edgesHz[m]) : T(1);

        AssignBinsToSegments(sampleRate, edgesHz);
    }

    template<typename T, std::size_t FftSize, std::size_t NumMelBands>
    void MelFilterbank<T, FftSize, NumMelBands>::AssignBinsToSegments(T sampleRate, const std::array<T, NumMelBands + 2>& edgesHz)
    {
        std::size_t j = 0;
        for (std::size_t k = 0; k < NumBins; ++k)
        {
            const T hz{ static_cast<T>(k) * sampleRate / static_cast<T>(FftSize) };

            while (j + 1 < edgesHz.size() && hz > edgesHz[j + 1])
                ++j;

            if (hz < edgesHz.front() || hz > edgesHz.back())
                segment[k] = outsideBands;
            else
            {
                segment[k] = static_cast<uint16_t>(j);
                risingWeight[k] = (hz - edgesHz[j]) / (edgesHz[j + 1] - edgesHz[j]);
            }
        }
    }

    template<typename T, std::size_t FftSize, std::size_t NumMelBands>
    OPTIMIZE_FOR_SPEED void MelFilterbank<T, FftSize, NumMelBands>::Apply(const infra::BoundedVector<T>& powerSpectrum, infra::BoundedVector<T>& melEnergies) const
    {
        really_assert(powerSpectrum.size() >= NumBins);

        melEnergies.resize(NumMelBands);
        for (std::size_t m = 0; m < NumMelBands; ++m)
            melEnergies[m] = T(0);

        for (std::size_t k = 0; k < NumBins; ++k)
        {
            const std::size_t j{ segment[k] };
            if (j == outsideBands)
                continue;

            if (j < NumMelBands)
                melEnergies[j] += risingWeight[k] * powerSpectrum[k];
            if (j >= 1)
                melEnergies[j - 1] += (T(1) - risingWeight[k]) * powerSpectrum[k];
        }

        for (std::size_t m = 0; m < NumMelBands; ++m)
            melEnergies[m] *= bandGain[m];
    }

    template<typename T, std::size_t FftSize, std::size_t NumMelBands>
    T MelFilterbank<T, FftSize, NumMelBands>::Weight(std::size_t band, std::size_t bin) const
    {
        really_assert(band < NumMelBands && bin < NumBins);

        const std::size_t j{ segment[bin] };
        if (j == band)
            return bandGain[band] * risingWeight[bin];
        if (j == band + 1)
            return bandGain[band] * (T(1) - risingWeight[bin]);
        return T(0);
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template float HzToMel<float>(float, MelScale);
    extern template float MelToHz<float>(float, MelScale);
    extern template class MelFilterbank<float, 256, 20>;
#endif
}
