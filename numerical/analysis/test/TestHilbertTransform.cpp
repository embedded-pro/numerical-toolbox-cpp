#include "numerical/analysis/FastFourierTransformRadix2Impl.hpp"
#include "numerical/analysis/HilbertTransform.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gmock/gmock.h"
#include <array>
#include <cmath>
#include <numbers>

namespace
{
    class TwiddleFactors64 : public analysis::TwiddleFactors<float, 32>
    {
    public:
        TwiddleFactors64()
        {
            for (std::size_t k{ 0 }; k < 32; ++k)
            {
                float angle{ -2.0f * std::numbers::pi_v<float> * static_cast<float>(k) / 64.0f };
                factors[k] = math::Complex<float>{ std::cos(angle), std::sin(angle) };
            }
        }

        math::Complex<float>& operator[](std::size_t n) override
        {
            return factors[n];
        }

    private:
        std::array<math::Complex<float>, 32> factors{};
    };

    class TestHilbertTransform : public ::testing::Test
    {
    public:
        static constexpr std::size_t N{ 64 };

        TwiddleFactors64 twiddle{};
        analysis::FastFourierTransformRadix2Impl<float, N> engine{ twiddle };
        analysis::AnalyticSignalFft<float, N> hilbert{ engine };
        analysis::HilbertFir<float, 31> fir{};

        typename infra::BoundedVector<float>::template WithMaxSize<N> input{};
    };
}

TEST_F(TestHilbertTransform, cosine_maps_to_sine_imag)
{
    static constexpr float f{ 3.0f };
    input.resize(N);
    for (std::size_t n{ 0 }; n < N; ++n)
        input[n] = std::cos(2.0f * std::numbers::pi_v<float> * f * static_cast<float>(n) / static_cast<float>(N));

    auto& a{ hilbert.Analytic(input) };

    for (std::size_t n{ 5 }; n < N - 5; ++n)
    {
        float expectedReal{ std::cos(2.0f * std::numbers::pi_v<float> * f * static_cast<float>(n) / static_cast<float>(N)) };
        float expectedImag{ std::sin(2.0f * std::numbers::pi_v<float> * f * static_cast<float>(n) / static_cast<float>(N)) };
        EXPECT_NEAR(a[n].Real(), expectedReal, 1e-4f);
        EXPECT_NEAR(a[n].Imaginary(), expectedImag, 1e-4f);
    }
}

TEST_F(TestHilbertTransform, envelope_of_am_signal_is_modulator)
{
    static constexpr float wc{ 2.0f * std::numbers::pi_v<float> * 12.0f / static_cast<float>(N) };
    static constexpr float wm{ 2.0f * std::numbers::pi_v<float> * 2.0f / static_cast<float>(N) };
    input.resize(N);
    for (std::size_t n{ 0 }; n < N; ++n)
    {
        float modulator{ 1.0f + 0.5f * std::cos(wm * static_cast<float>(n)) };
        input[n] = modulator * std::cos(wc * static_cast<float>(n));
    }

    auto& a{ hilbert.Analytic(input) };

    for (std::size_t n{ 5 }; n < N - 5; ++n)
    {
        float envelope{ analysis::AnalyticSignalFft<float, N>::InstantaneousAmplitude(a[n]) };
        float expected{ 1.0f + 0.5f * std::cos(wm * static_cast<float>(n)) };
        EXPECT_NEAR(envelope, expected, 5e-2f);
    }
}

TEST_F(TestHilbertTransform, analytic_energy_doubles_real_signal_energy)
{
    static constexpr float f{ 5.0f };
    input.resize(N);
    for (std::size_t n{ 0 }; n < N; ++n)
        input[n] = std::cos(2.0f * std::numbers::pi_v<float> * f * static_cast<float>(n) / static_cast<float>(N));

    auto& a{ hilbert.Analytic(input) };

    float realEnergy{ 0.0f };
    float analyticEnergy{ 0.0f };
    for (std::size_t n{ 0 }; n < N; ++n)
    {
        realEnergy += input[n] * input[n];
        analyticEnergy += a[n].Real() * a[n].Real() + a[n].Imaginary() * a[n].Imaginary();
    }

    EXPECT_NEAR(analyticEnergy, 2.0f * realEnergy, math::Tolerance<float>() * realEnergy);
}

TEST_F(TestHilbertTransform, instantaneous_frequency_of_tone_is_constant)
{
    static constexpr float f0{ 4.0f };
    static constexpr float expectedFreq{ f0 / static_cast<float>(N) };
    input.resize(N);
    for (std::size_t n{ 0 }; n < N; ++n)
        input[n] = std::cos(2.0f * std::numbers::pi_v<float> * f0 * static_cast<float>(n) / static_cast<float>(N));

    auto& a{ hilbert.Analytic(input) };

    static constexpr float ts{ 1.0f };
    for (std::size_t n{ 6 }; n < N - 5; ++n)
    {
        float phaseNow{ analysis::AnalyticSignalFft<float, N>::InstantaneousPhase(a[n]) };
        float phasePrev{ analysis::AnalyticSignalFft<float, N>::InstantaneousPhase(a[n - 1]) };
        float freq{ analysis::AnalyticSignalFft<float, N>::InstantaneousFrequency(phaseNow, phasePrev, ts) };
        EXPECT_NEAR(freq, expectedFreq, 1e-4f);
    }
}

TEST_F(TestHilbertTransform, instantaneous_frequency_tracks_chirp)
{
    static constexpr float f0{ 2.0f };
    static constexpr float rate{ 4.0f / static_cast<float>(N) };
    input.resize(N);
    for (std::size_t n{ 0 }; n < N; ++n)
    {
        float phase{ 2.0f * std::numbers::pi_v<float> * (f0 * static_cast<float>(n) + 0.5f * rate * static_cast<float>(n) * static_cast<float>(n)) / static_cast<float>(N) };
        input[n] = std::cos(phase);
    }

    auto& a{ hilbert.Analytic(input) };

    static constexpr float ts{ 1.0f };
    float prevFreq{ 0.0f };
    bool first{ true };
    for (std::size_t n{ 6 }; n < N - 5; ++n)
    {
        float phaseNow{ analysis::AnalyticSignalFft<float, N>::InstantaneousPhase(a[n]) };
        float phasePrev{ analysis::AnalyticSignalFft<float, N>::InstantaneousPhase(a[n - 1]) };
        float freq{ analysis::AnalyticSignalFft<float, N>::InstantaneousFrequency(phaseNow, phasePrev, ts) };
        if (!first)
            EXPECT_GE(freq, prevFreq - 5e-3f);
        prevFreq = freq;
        first = false;
    }
}

TEST_F(TestHilbertTransform, negative_frequencies_suppressed)
{
    static constexpr float f{ 5.0f };
    input.resize(N);
    for (std::size_t n{ 0 }; n < N; ++n)
        input[n] = std::cos(2.0f * std::numbers::pi_v<float> * f * static_cast<float>(n) / static_cast<float>(N));

    auto& a{ hilbert.Analytic(input) };

    std::array<math::Complex<float>, N> spectrum{};
    for (std::size_t k{ 0 }; k < N; ++k)
    {
        float re{ 0.0f };
        float im{ 0.0f };
        for (std::size_t n{ 0 }; n < N; ++n)
        {
            float angle{ -2.0f * std::numbers::pi_v<float> * static_cast<float>(k) * static_cast<float>(n) / static_cast<float>(N) };
            re += a[n].Real() * std::cos(angle) - a[n].Imaginary() * std::sin(angle);
            im += a[n].Real() * std::sin(angle) + a[n].Imaginary() * std::cos(angle);
        }
        spectrum[k] = math::Complex<float>{ re / static_cast<float>(N), im / static_cast<float>(N) };
    }

    for (std::size_t k{ N / 2 + 1 }; k < N; ++k)
    {
        float mag{ std::sqrt(spectrum[k].Real() * spectrum[k].Real() + spectrum[k].Imaginary() * spectrum[k].Imaginary()) };
        EXPECT_NEAR(mag, 0.0f, math::Tolerance<float>());
    }
}

TEST_F(TestHilbertTransform, dc_input_has_zero_hilbert)
{
    input.resize(N, 1.0f);

    auto& a{ hilbert.Analytic(input) };

    for (std::size_t n{ 0 }; n < N; ++n)
        EXPECT_NEAR(a[n].Imaginary(), 0.0f, math::Tolerance<float>());
}

TEST_F(TestHilbertTransform, nyquist_tone_has_zero_imaginary)
{
    input.resize(N);
    for (std::size_t n{ 0 }; n < N; ++n)
        input[n] = (n % 2 == 0) ? 1.0f : -1.0f;

    auto& a{ hilbert.Analytic(input) };

    for (std::size_t n{ 0 }; n < N; ++n)
        EXPECT_NEAR(a[n].Imaginary(), 0.0f, math::Tolerance<float>());
}

TEST_F(TestHilbertTransform, zero_signal_produces_zero_analytic)
{
    input.resize(N, 0.0f);

    auto& a{ hilbert.Analytic(input) };

    for (std::size_t n{ 0 }; n < N; ++n)
    {
        EXPECT_NEAR(a[n].Real(), 0.0f, math::Tolerance<float>());
        EXPECT_NEAR(a[n].Imaginary(), 0.0f, math::Tolerance<float>());
    }
}

TEST_F(TestHilbertTransform, analytic_is_deterministic)
{
    static constexpr float f{ 7.0f };
    input.resize(N);
    for (std::size_t n{ 0 }; n < N; ++n)
        input[n] = std::cos(2.0f * std::numbers::pi_v<float> * f * static_cast<float>(n) / static_cast<float>(N));

    auto& a1{ hilbert.Analytic(input) };
    std::array<float, N> reals1{};
    std::array<float, N> imags1{};
    for (std::size_t n{ 0 }; n < N; ++n)
    {
        reals1[n] = a1[n].Real();
        imags1[n] = a1[n].Imaginary();
    }

    TwiddleFactors64 twiddle2{};
    analysis::FastFourierTransformRadix2Impl<float, N> engine2{ twiddle2 };
    analysis::AnalyticSignalFft<float, N> hilbert2{ engine2 };

    typename infra::BoundedVector<float>::template WithMaxSize<N> input2{};
    input2.resize(N);
    for (std::size_t n{ 0 }; n < N; ++n)
        input2[n] = input[n];

    auto& a2{ hilbert2.Analytic(input2) };

    for (std::size_t n{ 0 }; n < N; ++n)
    {
        EXPECT_FLOAT_EQ(a2[n].Real(), reals1[n]);
        EXPECT_FLOAT_EQ(a2[n].Imaginary(), imags1[n]);
    }
}

TEST_F(TestHilbertTransform, analytic_is_linear)
{
    static constexpr float fa{ 3.0f };
    static constexpr float fb{ 7.0f };
    static constexpr float alpha{ 2.5f };
    static constexpr float beta{ -1.5f };

    typename infra::BoundedVector<float>::template WithMaxSize<N> xa{};
    typename infra::BoundedVector<float>::template WithMaxSize<N> xb{};
    typename infra::BoundedVector<float>::template WithMaxSize<N> xsum{};
    xa.resize(N);
    xb.resize(N);
    xsum.resize(N);

    for (std::size_t n{ 0 }; n < N; ++n)
    {
        xa[n] = std::cos(2.0f * std::numbers::pi_v<float> * fa * static_cast<float>(n) / static_cast<float>(N));
        xb[n] = std::cos(2.0f * std::numbers::pi_v<float> * fb * static_cast<float>(n) / static_cast<float>(N));
        xsum[n] = alpha * xa[n] + beta * xb[n];
    }

    TwiddleFactors64 twA{};
    analysis::FastFourierTransformRadix2Impl<float, N> engA{ twA };
    analysis::AnalyticSignalFft<float, N> hA{ engA };

    TwiddleFactors64 twB{};
    analysis::FastFourierTransformRadix2Impl<float, N> engB{ twB };
    analysis::AnalyticSignalFft<float, N> hB{ engB };

    auto& aa{ hA.Analytic(xa) };
    std::array<float, N> combinedReal{};
    std::array<float, N> combinedImag{};
    for (std::size_t n{ 0 }; n < N; ++n)
    {
        combinedReal[n] = alpha * aa[n].Real();
        combinedImag[n] = alpha * aa[n].Imaginary();
    }

    auto& ab{ hB.Analytic(xb) };
    for (std::size_t n{ 0 }; n < N; ++n)
    {
        combinedReal[n] += beta * ab[n].Real();
        combinedImag[n] += beta * ab[n].Imaginary();
    }

    auto& asum{ hilbert.Analytic(xsum) };

    for (std::size_t n{ 2 }; n < N - 2; ++n)
    {
        EXPECT_NEAR(asum[n].Real(), combinedReal[n], math::Tolerance<float>());
        EXPECT_NEAR(asum[n].Imaginary(), combinedImag[n], math::Tolerance<float>());
    }
}

TEST_F(TestHilbertTransform, fir_quadrature_at_passband_frequency)
{
    static constexpr float f{ 4.0f };
    static constexpr std::size_t FirTaps{ 31 };
    static constexpr std::size_t centerTap{ (FirTaps - 1) / 2 };
    static constexpr std::size_t warmup{ FirTaps };

    analysis::HilbertFir<float, FirTaps> firFilter{};

    for (std::size_t s{ 0 }; s < warmup; ++s)
    {
        float val{ std::cos(2.0f * std::numbers::pi_v<float> * f * static_cast<float>(s) / static_cast<float>(N)) };
        firFilter.Filter(val);
    }

    float maxErr{ 0.0f };
    for (std::size_t s{ warmup }; s < warmup + 20; ++s)
    {
        float val{ std::cos(2.0f * std::numbers::pi_v<float> * f * static_cast<float>(s) / static_cast<float>(N)) };
        auto out{ firFilter.Filter(val) };
        std::size_t realSample{ s - centerTap };
        float expectedImag{ std::sin(2.0f * std::numbers::pi_v<float> * f * static_cast<float>(realSample) / static_cast<float>(N)) };
        float err{ std::abs(out.Imaginary() - expectedImag) };
        if (err > maxErr)
            maxErr = err;
    }
    EXPECT_NEAR(maxErr, 0.0f, 5e-2f);
}
