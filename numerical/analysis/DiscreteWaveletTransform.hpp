#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "infra/util/BoundedVector.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include <array>
#include <cstddef>
#include <type_traits>

namespace analysis
{
    template<typename T, std::size_t Taps>
    class WaveletFilters
    {
        static_assert(std::is_floating_point_v<T>, "WaveletFilters supports floating-point types only");

    public:
        std::array<T, Taps> lowAnalysis{};
        std::array<T, Taps> highAnalysis{};
        std::array<T, Taps> lowSynthesis{};
        std::array<T, Taps> highSynthesis{};
    };

    template<typename T, std::size_t Taps>
    WaveletFilters<T, Taps> MakeHaar()
    {
        static_assert(Taps == 2, "Haar wavelet requires Taps == 2");
        static_assert(std::is_floating_point_v<T>, "MakeHaar supports floating-point types only");

        constexpr T inv_sqrt2 = T{ 0.7071067811865476 };
        WaveletFilters<T, 2> f{};
        f.lowAnalysis[0] = inv_sqrt2;
        f.lowAnalysis[1] = inv_sqrt2;
        f.highAnalysis[0] = inv_sqrt2;
        f.highAnalysis[1] = -inv_sqrt2;
        f.lowSynthesis[0] = inv_sqrt2;
        f.lowSynthesis[1] = inv_sqrt2;
        f.highSynthesis[0] = -inv_sqrt2;
        f.highSynthesis[1] = inv_sqrt2;
        return f;
    }

    template<typename T, std::size_t Taps>
    WaveletFilters<T, Taps> MakeDaubechies2()
    {
        static_assert(Taps == 4, "Daubechies-2 wavelet requires Taps == 4");
        static_assert(std::is_floating_point_v<T>, "MakeDaubechies2 supports floating-point types only");

        WaveletFilters<T, 4> f{};
        f.lowAnalysis[0] = T{ 0.4829629131445341 };
        f.lowAnalysis[1] = T{ 0.8365163037378079 };
        f.lowAnalysis[2] = T{ 0.2241438680420134 };
        f.lowAnalysis[3] = T{ -0.1294095225512604 };

        for (std::size_t k = 0; k < 4; ++k)
        {
            T sign = ((k % 2) == 0) ? T{ 1 } : T{ -1 };
            f.highAnalysis[k] = sign * f.lowAnalysis[3 - k];
        }

        f.lowSynthesis[0] = f.lowAnalysis[3];
        f.lowSynthesis[1] = f.lowAnalysis[2];
        f.lowSynthesis[2] = f.lowAnalysis[1];
        f.lowSynthesis[3] = f.lowAnalysis[0];

        for (std::size_t k = 0; k < 4; ++k)
        {
            T sign = (((k + 1) % 2) == 0) ? T{ 1 } : T{ -1 };
            f.highSynthesis[k] = sign * f.lowSynthesis[3 - k];
        }

        return f;
    }

    template<typename T, std::size_t N, std::size_t Levels, std::size_t Taps>
    class DiscreteWaveletTransform
    {
        static_assert(std::is_floating_point_v<T>, "DiscreteWaveletTransform supports floating-point types only");
        static_assert(N > 0, "DiscreteWaveletTransform N must be > 0");
        static_assert(Levels > 0, "DiscreteWaveletTransform Levels must be > 0");
        static_assert(Taps >= 2, "DiscreteWaveletTransform Taps must be >= 2");

    public:
        using Signal = typename infra::BoundedVector<T>::template WithMaxSize<N>;

        explicit DiscreteWaveletTransform(WaveletFilters<T, Taps> filtersIn);

        OPTIMIZE_FOR_SPEED void Forward(const Signal& x, Signal& coeffs);
        void Inverse(const Signal& coeffs, Signal& x);
        std::size_t LevelOffset(std::size_t level) const;

    private:
        WaveletFilters<T, Taps> filters;
        typename infra::BoundedVector<T>::template WithMaxSize<N> workBuf;
        typename infra::BoundedVector<T>::template WithMaxSize<N / 2> caBuf;
        typename infra::BoundedVector<T>::template WithMaxSize<N / 2> cdBuf;

        void AnalysisStep(const Signal& s, std::size_t len,
            typename infra::BoundedVector<T>::template WithMaxSize<N / 2>& ca,
            typename infra::BoundedVector<T>::template WithMaxSize<N / 2>& cd);

        void SynthesisStep(
            const typename infra::BoundedVector<T>::template WithMaxSize<N / 2>& ca,
            const typename infra::BoundedVector<T>::template WithMaxSize<N / 2>& cd,
            Signal& out, std::size_t outLen);
    };

    template<typename T, std::size_t N, std::size_t Levels, std::size_t Taps>
    DiscreteWaveletTransform<T, N, Levels, Taps>::DiscreteWaveletTransform(WaveletFilters<T, Taps> filtersIn)
        : filters{ filtersIn }
    {
        workBuf.resize(N, T{ 0 });
        caBuf.resize(N / 2, T{ 0 });
        cdBuf.resize(N / 2, T{ 0 });
    }

    template<typename T, std::size_t N, std::size_t Levels, std::size_t Taps>
    void DiscreteWaveletTransform<T, N, Levels, Taps>::AnalysisStep(
        const Signal& s, std::size_t len,
        typename infra::BoundedVector<T>::template WithMaxSize<N / 2>& ca,
        typename infra::BoundedVector<T>::template WithMaxSize<N / 2>& cd)
    {
        std::size_t half = len / 2;
        ca.clear();
        ca.resize(half, T{ 0 });
        cd.clear();
        cd.resize(half, T{ 0 });

        for (std::size_t i = 0; i < half; ++i)
        {
            T sumA{ 0 };
            T sumD{ 0 };
            for (std::size_t k = 0; k < Taps; ++k)
            {
                std::size_t idx = (2 * i + k) % len;
                sumA += filters.lowAnalysis[k] * s[idx];
                sumD += filters.highAnalysis[k] * s[idx];
            }
            ca[i] = sumA;
            cd[i] = sumD;
        }
    }

    template<typename T, std::size_t N, std::size_t Levels, std::size_t Taps>
    void DiscreteWaveletTransform<T, N, Levels, Taps>::SynthesisStep(
        const typename infra::BoundedVector<T>::template WithMaxSize<N / 2>& ca,
        const typename infra::BoundedVector<T>::template WithMaxSize<N / 2>& cd,
        Signal& out, std::size_t outLen)
    {
        std::size_t half = outLen / 2;
        out.clear();
        out.resize(outLen, T{ 0 });

        for (std::size_t n = 0; n < outLen; ++n)
        {
            T sumA{ 0 };
            T sumD{ 0 };
            for (std::size_t i = 0; i < half; ++i)
            {
                std::size_t fIdx = (n + outLen - 2 * i) % outLen;
                if (fIdx < Taps)
                {
                    sumA += filters.lowAnalysis[fIdx] * ca[i];
                    sumD += filters.highAnalysis[fIdx] * cd[i];
                }
            }
            out[n] = sumA + sumD;
        }
    }

    template<typename T, std::size_t N, std::size_t Levels, std::size_t Taps>
    OPTIMIZE_FOR_SPEED void DiscreteWaveletTransform<T, N, Levels, Taps>::Forward(const Signal& x, Signal& coeffs)
    {
        coeffs.clear();
        coeffs.resize(N, T{ 0 });

        for (std::size_t i = 0; i < N; ++i)
            workBuf[i] = x[i];

        std::size_t curLen = N;

        for (std::size_t lvl = 0; lvl < Levels; ++lvl)
        {
            AnalysisStep(workBuf, curLen, caBuf, cdBuf);

            std::size_t half = curLen / 2;
            std::size_t detailOffset = LevelOffset(lvl);
            for (std::size_t i = 0; i < half; ++i)
                coeffs[detailOffset + i] = cdBuf[i];

            for (std::size_t i = 0; i < half; ++i)
                workBuf[i] = caBuf[i];

            curLen = half;
        }

        for (std::size_t i = 0; i < curLen; ++i)
            coeffs[N - curLen + i] = workBuf[i];
    }

    template<typename T, std::size_t N, std::size_t Levels, std::size_t Taps>
    void DiscreteWaveletTransform<T, N, Levels, Taps>::Inverse(const Signal& coeffs, Signal& x)
    {
        std::size_t approxLen = N;
        for (std::size_t lvl = 0; lvl < Levels; ++lvl)
            approxLen /= 2;

        for (std::size_t i = 0; i < approxLen; ++i)
            workBuf[i] = coeffs[N - approxLen + i];

        std::size_t curLen = approxLen;

        for (std::size_t lvl = Levels; lvl > 0; --lvl)
        {
            std::size_t detailOffset = LevelOffset(lvl - 1);
            std::size_t half = curLen;
            std::size_t outLen = curLen * 2;

            caBuf.clear();
            caBuf.resize(half, T{ 0 });
            cdBuf.clear();
            cdBuf.resize(half, T{ 0 });

            for (std::size_t i = 0; i < half; ++i)
            {
                caBuf[i] = workBuf[i];
                cdBuf[i] = coeffs[detailOffset + i];
            }

            SynthesisStep(caBuf, cdBuf, workBuf, outLen);
            curLen = outLen;
        }

        x.clear();
        x.resize(N, T{ 0 });
        for (std::size_t i = 0; i < N; ++i)
            x[i] = workBuf[i];
    }

    template<typename T, std::size_t N, std::size_t Levels, std::size_t Taps>
    std::size_t DiscreteWaveletTransform<T, N, Levels, Taps>::LevelOffset(std::size_t level) const
    {
        std::size_t offset = 0;
        std::size_t len = N;
        for (std::size_t lvl = 0; lvl < level; ++lvl)
        {
            len /= 2;
            offset += len;
        }
        return offset;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class WaveletFilters<float, 2>;
    extern template class WaveletFilters<float, 4>;
    extern template class DiscreteWaveletTransform<float, 16, 3, 2>;
    extern template class DiscreteWaveletTransform<float, 16, 3, 4>;
#endif
}
