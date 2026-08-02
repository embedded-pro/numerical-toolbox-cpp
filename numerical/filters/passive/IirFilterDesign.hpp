#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/filters/passive/BiquadCascade.hpp"
#include "numerical/math/ComplexNumber.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <type_traits>

namespace filters::passive
{
    enum class Prototype
    {
        Butterworth,
        ChebyshevI
    };

    enum class Kind
    {
        LowPass,
        HighPass,
        BandPass,
        BandStop
    };

    template<typename T, std::size_t MaxOrder>
    class IirFilterDesign
    {
        static_assert(std::is_floating_point_v<T>, "IirFilterDesign supports floating-point types");
        static_assert(MaxOrder > 0 && MaxOrder <= 16, "MaxOrder must be in [1,16]");

    public:
        IirFilterDesign() noexcept = default;

        OPTIMIZE_FOR_SPEED std::size_t Design(Prototype proto, Kind kind, std::size_t order,
            T cutoffHz, T sampleHz, T rippleDb = T{ 1 }) noexcept;

        BiquadCoeffs<T> Section(std::size_t i) const noexcept;

    private:
        static constexpr std::size_t kMaxSections{ (MaxOrder + 1) / 2 };

        using ComplexT = math::Complex<T>;

        std::array<BiquadCoeffs<T>, kMaxSections> sections{};
        std::size_t sectionCount{ 0 };

        static void ButterPoles(std::size_t order, std::array<ComplexT, 16>& poles) noexcept;
        static void ChebyPoles(std::size_t order, T rippleDb, std::array<ComplexT, 16>& poles) noexcept;

        static ComplexT BilinearS2Z(ComplexT s, T fs) noexcept;

        static std::size_t SelectPole(const std::array<ComplexT, 32>& az, std::size_t numA,
            const std::array<bool, 32>& used, std::size_t start) noexcept;
        static void MarkConjugate(const std::array<ComplexT, 32>& az, std::size_t numA,
            std::array<bool, 32>& used, ComplexT p) noexcept;
        static void PairPole(const std::array<ComplexT, 32>& az, std::size_t numA,
            std::array<bool, 32>& used, std::size_t start, T& a1, T& a2) noexcept;

        static void PairComplexZero(ComplexT z, T& b1, T& b2) noexcept;
        static void PairRealZeros(const std::array<ComplexT, 32>& bz, std::size_t numB,
            std::array<bool, 32>& used, std::size_t& bi, ComplexT z1, T& b1, T& b2) noexcept;
        static void PairZero(const std::array<ComplexT, 32>& bz, std::size_t numB,
            std::array<bool, 32>& used, std::size_t& bi, T& b0, T& b1, T& b2) noexcept;

        static void BuildSections(const std::array<ComplexT, 32>& bz, std::size_t numB,
            const std::array<ComplexT, 32>& az, std::size_t numA, T gain,
            std::array<BiquadCoeffs<T>, kMaxSections>& out, std::size_t& count) noexcept;

        OPTIMIZE_FOR_SPEED std::size_t DesignLowPass(std::size_t order, T wc, T fs,
            const std::array<ComplexT, 16>& protoPoles) noexcept;
        static void HighPassRoots(std::size_t order, T wc, T fs,
            const std::array<ComplexT, 16>& protoPoles,
            std::array<ComplexT, 32>& az, std::array<ComplexT, 32>& bz) noexcept;
        static T HighPassGain(std::size_t order, const std::array<ComplexT, 32>& az) noexcept;
        OPTIMIZE_FOR_SPEED std::size_t DesignHighPass(std::size_t order, T wc, T fs,
            const std::array<ComplexT, 16>& protoPoles) noexcept;
        OPTIMIZE_FOR_SPEED std::size_t DesignBandPass(std::size_t order, T wc, T fs,
            const std::array<ComplexT, 16>& protoPoles) noexcept;
        OPTIMIZE_FOR_SPEED std::size_t DesignBandStop(std::size_t order, T wc, T fs,
            const std::array<ComplexT, 16>& protoPoles) noexcept;
    };

    template<typename T, std::size_t MaxOrder>
    void IirFilterDesign<T, MaxOrder>::ButterPoles(std::size_t order, std::array<ComplexT, 16>& poles) noexcept
    {
        for (std::size_t k{ 0 }; k < order; ++k)
        {
            const T theta{ std::numbers::pi_v<T> * (T{ 2 } * static_cast<T>(k) + static_cast<T>(order) + T{ 1 }) / (T{ 2 } * static_cast<T>(order)) };
            poles[k] = { std::cos(theta), std::sin(theta) };
        }
    }

    template<typename T, std::size_t MaxOrder>
    void IirFilterDesign<T, MaxOrder>::ChebyPoles(std::size_t order, T rippleDb, std::array<ComplexT, 16>& poles) noexcept
    {
        const T epsilon{ std::sqrt(std::pow(T{ 10 }, rippleDb / T{ 10 }) - T{ 1 }) };
        const T n{ static_cast<T>(order) };
        const T asinh_val{ std::log(T{ 1 } / epsilon + std::sqrt(T{ 1 } / (epsilon * epsilon) + T{ 1 })) };
        const T sigma0{ std::sinh(asinh_val / n) };
        const T omega0{ std::cosh(asinh_val / n) };

        for (std::size_t k{ 0 }; k < order; ++k)
        {
            const T theta{ std::numbers::pi_v<T> * (T{ 2 } * static_cast<T>(k) + T{ 1 }) / (T{ 2 } * n) };
            poles[k] = { -sigma0 * std::sin(theta), omega0 * std::cos(theta) };
        }
    }

    template<typename T, std::size_t MaxOrder>
    typename IirFilterDesign<T, MaxOrder>::ComplexT
    IirFilterDesign<T, MaxOrder>::BilinearS2Z(ComplexT s, T fs) noexcept
    {
        const T twoFs{ T{ 2 } * fs };
        const ComplexT num{ T{ 1 } + s.Real() / twoFs, s.Imaginary() / twoFs };
        const ComplexT den{ T{ 1 } - s.Real() / twoFs, -s.Imaginary() / twoFs };
        return num / den;
    }

    template<typename T, std::size_t MaxOrder>
    std::size_t IirFilterDesign<T, MaxOrder>::SelectPole(const std::array<ComplexT, 32>& az, std::size_t numA,
        const std::array<bool, 32>& used, std::size_t start) noexcept
    {
        std::size_t idx{ start };
        for (std::size_t j{ start }; j < numA; ++j)
        {
            if (!used[j] && std::abs(az[j].Imaginary()) > std::abs(az[idx].Imaginary()))
                idx = j;
        }
        return idx;
    }

    template<typename T, std::size_t MaxOrder>
    void IirFilterDesign<T, MaxOrder>::MarkConjugate(const std::array<ComplexT, 32>& az, std::size_t numA,
        std::array<bool, 32>& used, ComplexT p) noexcept
    {
        std::size_t conjIdx{ numA };
        T bestDist{ T{ 1e30 } };
        for (std::size_t j{ 0 }; j < numA; ++j)
        {
            if (!used[j])
            {
                const T dist{ std::abs(az[j].Real() - p.Real()) + std::abs(az[j].Imaginary() + p.Imaginary()) };
                if (dist < bestDist)
                {
                    bestDist = dist;
                    conjIdx = j;
                }
            }
        }
        if (conjIdx < numA)
            used[conjIdx] = true;
    }

    template<typename T, std::size_t MaxOrder>
    void IirFilterDesign<T, MaxOrder>::PairPole(const std::array<ComplexT, 32>& az, std::size_t numA,
        std::array<bool, 32>& used, std::size_t start, T& a1, T& a2) noexcept
    {
        const std::size_t idx{ SelectPole(az, numA, used, start) };
        const ComplexT p{ az[idx] };
        used[idx] = true;

        if (std::abs(p.Imaginary()) > T{ 1e-6 })
        {
            MarkConjugate(az, numA, used, p);
            a1 = -T{ 2 } * p.Real();
            a2 = p.Real() * p.Real() + p.Imaginary() * p.Imaginary();
        }
        else
        {
            a1 = -p.Real();
            a2 = T{ 0 };
        }
    }

    template<typename T, std::size_t MaxOrder>
    void IirFilterDesign<T, MaxOrder>::PairComplexZero(ComplexT z, T& b1, T& b2) noexcept
    {
        b1 = -T{ 2 } * z.Real();
        b2 = z.Real() * z.Real() + z.Imaginary() * z.Imaginary();
    }

    template<typename T, std::size_t MaxOrder>
    void IirFilterDesign<T, MaxOrder>::PairRealZeros(const std::array<ComplexT, 32>& bz, std::size_t numB,
        std::array<bool, 32>& used, std::size_t& bi, ComplexT z1, T& b1, T& b2) noexcept
    {
        if (bi < numB && !used[bi])
        {
            const ComplexT z2{ bz[bi] };
            used[bi] = true;
            ++bi;
            b1 = -(z1.Real() + z2.Real());
            b2 = z1.Real() * z2.Real();
        }
        else
        {
            b1 = -z1.Real();
            b2 = T{ 0 };
        }
    }

    template<typename T, std::size_t MaxOrder>
    void IirFilterDesign<T, MaxOrder>::PairZero(const std::array<ComplexT, 32>& bz, std::size_t numB,
        std::array<bool, 32>& used, std::size_t& bi, T& b0, T& b1, T& b2) noexcept
    {
        b0 = T{ 1 };
        b1 = T{ 0 };
        b2 = T{ 0 };

        if (bi >= numB || used[bi])
            return;

        const ComplexT z1{ bz[bi] };
        used[bi] = true;
        ++bi;

        if (std::abs(z1.Imaginary()) > T{ 1e-6 } && bi < numB && !used[bi])
        {
            used[bi] = true;
            ++bi;
            PairComplexZero(z1, b1, b2);
        }
        else if (std::abs(z1.Imaginary()) <= T{ 1e-6 })
        {
            PairRealZeros(bz, numB, used, bi, z1, b1, b2);
        }
    }

    template<typename T, std::size_t MaxOrder>
    void IirFilterDesign<T, MaxOrder>::BuildSections(const std::array<ComplexT, 32>& bz, std::size_t numB,
        const std::array<ComplexT, 32>& az, std::size_t numA, T gain,
        std::array<BiquadCoeffs<T>, kMaxSections>& out, std::size_t& count) noexcept
    {
        count = 0;
        std::size_t bi{ 0 };
        std::size_t ai{ 0 };
        std::array<bool, 32> bUsed{};
        std::array<bool, 32> aUsed{};

        while (ai < numA && count < kMaxSections)
        {
            T a1{};
            T a2{};
            T b0{};
            T b1{};
            T b2{};

            PairPole(az, numA, aUsed, ai, a1, a2);
            PairZero(bz, numB, bUsed, bi, b0, b1, b2);

            const T sectionGain{ (count == 0) ? gain : T{ 1 } };

            if (std::abs(a2) < T{ 1e-10 })
                out[count] = { sectionGain * b0, sectionGain * b1, T{ 0 }, a1, T{ 0 } };
            else
                out[count] = { sectionGain * b0, sectionGain * b1, sectionGain * b2, a1, a2 };

            ++count;

            while (ai < numA && aUsed[ai])
                ++ai;
        }
    }

    template<typename T, std::size_t MaxOrder>
    OPTIMIZE_FOR_SPEED std::size_t IirFilterDesign<T, MaxOrder>::DesignLowPass(std::size_t order, T wc, T fs,
        const std::array<ComplexT, 16>& protoPoles) noexcept
    {
        std::array<ComplexT, 32> az{};
        std::array<ComplexT, 32> bz{};
        const std::size_t numA{ order };
        const std::size_t numB{ order };

        for (std::size_t k{ 0 }; k < order; ++k)
        {
            const ComplexT scaled{ protoPoles[k].Real() * wc, protoPoles[k].Imaginary() * wc };
            az[k] = BilinearS2Z(scaled, fs);
            bz[k] = { -T{ 1 }, T{ 0 } };
        }

        T dcNum{ T{ 1 } };
        T dcDen{ T{ 1 } };
        for (std::size_t k{ 0 }; k < order; ++k)
        {
            dcNum *= T{ 2 };
            const T pre{ T{ 1 } - az[k].Real() };
            const T pim{ az[k].Imaginary() };
            dcDen *= std::sqrt(pre * pre + pim * pim);
        }

        const T rawGain{ (dcDen > T{ 0 }) ? (dcNum / dcDen) : T{ 1 } };
        const T gain{ (rawGain > T{ 0 }) ? (T{ 1 } / rawGain) : T{ 1 } };

        BuildSections(bz, numB, az, numA, gain, sections, sectionCount);
        return sectionCount;
    }

    template<typename T, std::size_t MaxOrder>
    void IirFilterDesign<T, MaxOrder>::HighPassRoots(std::size_t order, T wc, T fs,
        const std::array<ComplexT, 16>& protoPoles,
        std::array<ComplexT, 32>& az, std::array<ComplexT, 32>& bz) noexcept
    {
        for (std::size_t k{ 0 }; k < order; ++k)
        {
            const T mag2{ protoPoles[k].Real() * protoPoles[k].Real() + protoPoles[k].Imaginary() * protoPoles[k].Imaginary() };
            const ComplexT hpPole{ wc / protoPoles[k].Real(), -wc * protoPoles[k].Imaginary() / mag2 };
            az[k] = BilinearS2Z(hpPole, fs);
            bz[k] = { T{ 1 }, T{ 0 } };
        }
    }

    template<typename T, std::size_t MaxOrder>
    T IirFilterDesign<T, MaxOrder>::HighPassGain(std::size_t order, const std::array<ComplexT, 32>& az) noexcept
    {
        T nyqNum{ T{ 1 } };
        T nyqDen{ T{ 1 } };
        for (std::size_t k{ 0 }; k < order; ++k)
        {
            nyqNum *= T{ 2 };
            const T dre{ T{ 1 } + az[k].Real() };
            const T dim{ az[k].Imaginary() };
            nyqDen *= std::sqrt(dre * dre + dim * dim);
        }
        return (nyqDen > T{ 0 }) ? (T{ 1 } / (nyqNum / nyqDen)) : T{ 1 };
    }

    template<typename T, std::size_t MaxOrder>
    OPTIMIZE_FOR_SPEED std::size_t IirFilterDesign<T, MaxOrder>::DesignHighPass(std::size_t order, T wc, T fs,
        const std::array<ComplexT, 16>& protoPoles) noexcept
    {
        std::array<ComplexT, 32> az{};
        std::array<ComplexT, 32> bz{};
        const std::size_t numA{ order };
        const std::size_t numB{ order };

        HighPassRoots(order, wc, fs, protoPoles, az, bz);
        const T gain{ HighPassGain(order, az) };

        BuildSections(bz, numB, az, numA, gain, sections, sectionCount);
        return sectionCount;
    }

    template<typename T, std::size_t MaxOrder>
    OPTIMIZE_FOR_SPEED std::size_t IirFilterDesign<T, MaxOrder>::DesignBandPass(std::size_t order, T wc, T fs,
        const std::array<ComplexT, 16>& protoPoles) noexcept
    {
        const T bw{ wc };
        const T wc0{ wc };
        std::array<ComplexT, 32> az{};
        std::array<ComplexT, 32> bz{};
        std::size_t numA{ 0 };
        std::size_t numB{ 0 };

        for (std::size_t k{ 0 }; k < order; ++k)
        {
            const ComplexT lp{ protoPoles[k].Real() * bw / T{ 2 }, protoPoles[k].Imaginary() * bw / T{ 2 } };
            const T discRe{ lp.Real() * lp.Real() - lp.Imaginary() * lp.Imaginary() - wc0 * wc0 };
            const T discIm{ T{ 2 } * lp.Real() * lp.Imaginary() };
            const T discMag{ std::sqrt(std::sqrt(discRe * discRe + discIm * discIm)) };
            const T discAngle{ std::atan2(discIm, discRe) / T{ 2 } };
            const ComplexT sqrtDisc{ discMag * std::cos(discAngle), discMag * std::sin(discAngle) };

            az[numA++] = BilinearS2Z(lp + sqrtDisc, fs);
            az[numA++] = BilinearS2Z(lp - sqrtDisc, fs);
            bz[numB++] = { T{ 1 }, T{ 0 } };
            bz[numB++] = { -T{ 1 }, T{ 0 } };
        }

        const T gain{ T{ 1 } };
        BuildSections(bz, numB, az, numA, gain, sections, sectionCount);
        return sectionCount;
    }

    template<typename T, std::size_t MaxOrder>
    OPTIMIZE_FOR_SPEED std::size_t IirFilterDesign<T, MaxOrder>::DesignBandStop(std::size_t order, T wc, T fs,
        const std::array<ComplexT, 16>& protoPoles) noexcept
    {
        const T bw{ wc };
        const T wc0{ wc };
        std::array<ComplexT, 32> az{};
        std::array<ComplexT, 32> bz{};
        std::size_t numA{ 0 };
        std::size_t numB{ 0 };

        for (std::size_t k{ 0 }; k < order; ++k)
        {
            const T mag2{ protoPoles[k].Real() * protoPoles[k].Real() + protoPoles[k].Imaginary() * protoPoles[k].Imaginary() };
            const ComplexT invLp{ protoPoles[k].Real() / mag2, -protoPoles[k].Imaginary() / mag2 };
            const ComplexT bsLp{ invLp.Real() * bw / T{ 2 }, invLp.Imaginary() * bw / T{ 2 } };

            const T discRe{ bsLp.Real() * bsLp.Real() - bsLp.Imaginary() * bsLp.Imaginary() - wc0 * wc0 };
            const T discIm{ T{ 2 } * bsLp.Real() * bsLp.Imaginary() };
            const T discMag{ std::sqrt(std::sqrt(discRe * discRe + discIm * discIm)) };
            const T discAngle{ std::atan2(discIm, discRe) / T{ 2 } };
            const ComplexT sqrtDisc{ discMag * std::cos(discAngle), discMag * std::sin(discAngle) };

            az[numA++] = BilinearS2Z(bsLp + sqrtDisc, fs);
            az[numA++] = BilinearS2Z(bsLp - sqrtDisc, fs);
            bz[numB++] = BilinearS2Z({ T{ 0 }, wc0 }, fs);
            bz[numB++] = BilinearS2Z({ T{ 0 }, -wc0 }, fs);
        }

        const T gain{ T{ 1 } };
        BuildSections(bz, numB, az, numA, gain, sections, sectionCount);
        return sectionCount;
    }

    template<typename T, std::size_t MaxOrder>
    OPTIMIZE_FOR_SPEED std::size_t IirFilterDesign<T, MaxOrder>::Design(Prototype proto, Kind kind,
        std::size_t order, T cutoffHz, T sampleHz, T rippleDb) noexcept
    {
        sectionCount = 0;
        if (order == 0 || order > MaxOrder || cutoffHz <= T{ 0 } || sampleHz <= T{ 0 } || cutoffHz >= sampleHz / T{ 2 })
            return 0;

        std::array<ComplexT, 16> protoPoles{};

        if (proto == Prototype::Butterworth)
            ButterPoles(order, protoPoles);
        else
            ChebyPoles(order, rippleDb, protoPoles);

        const T wc{ T{ 2 } * sampleHz * std::tan(std::numbers::pi_v<T> * cutoffHz / sampleHz) };

        switch (kind)
        {
            case Kind::LowPass:
                return DesignLowPass(order, wc, sampleHz, protoPoles);
            case Kind::HighPass:
                return DesignHighPass(order, wc, sampleHz, protoPoles);
            case Kind::BandPass:
                return DesignBandPass(order, wc, sampleHz, protoPoles);
            case Kind::BandStop:
                return DesignBandStop(order, wc, sampleHz, protoPoles);
            default:
                return 0;
        }
    }

    template<typename T, std::size_t MaxOrder>
    BiquadCoeffs<T> IirFilterDesign<T, MaxOrder>::Section(std::size_t i) const noexcept
    {
        if (i < sectionCount)
            return sections[i];
        return BiquadCoeffs<T>{};
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class IirFilterDesign<float, 8>;
#endif
}
