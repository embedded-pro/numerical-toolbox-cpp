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
        EXPECT_NEAR(a[n].Real(), expectedReal, 1e-2f);
        EXPECT_NEAR(a[n].Imaginary(), expectedImag, 1e-2f);
    }
}

TEST_F(TestHilbertTransform, analytic_real_part_equals_input)
{
    static constexpr float f{ 5.0f };
    input.resize(N);
    for (std::size_t n{ 0 }; n < N; ++n)
        input[n] = std::cos(2.0f * std::numbers::pi_v<float> * f * static_cast<float>(n) / static_cast<float>(N));

    auto& a{ hilbert.Analytic(input) };

    for (std::size_t n{ 5 }; n < N - 5; ++n)
        EXPECT_NEAR(a[n].Real(), input[n], 1e-2f);
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
        EXPECT_NEAR(freq, expectedFreq, 1e-2f);
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
    float prevFreq{ -1.0f };
    for (std::size_t n{ 6 }; n < N - 5; ++n)
    {
        float phaseNow{ analysis::AnalyticSignalFft<float, N>::InstantaneousPhase(a[n]) };
        float phasePrev{ analysis::AnalyticSignalFft<float, N>::InstantaneousPhase(a[n - 1]) };
        float freq{ analysis::AnalyticSignalFft<float, N>::InstantaneousFrequency(phaseNow, phasePrev, ts) };
        if (prevFreq >= 0.0f)
            EXPECT_GE(freq, prevFreq - 5e-3f);
        prevFreq = freq;
    }
}

TEST_F(TestHilbertTransform, negative_frequencies_are_zeroed)
{
    static constexpr float f{ 5.0f };
    input.resize(N);
    for (std::size_t n{ 0 }; n < N; ++n)
        input[n] = std::cos(2.0f * std::numbers::pi_v<float> * f * static_cast<float>(n) / static_cast<float>(N));

    auto& a{ hilbert.Analytic(input) };

    float positiveEnergy{ 0.0f };
    for (std::size_t n{ 0 }; n < N; ++n)
        positiveEnergy += a[n].Real() * a[n].Real() + a[n].Imaginary() * a[n].Imaginary();

    typename infra::BoundedVector<math::Complex<float>>::template WithMaxSize<N> spectrum{};
    spectrum.resize(N);
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
        EXPECT_NEAR(mag, 0.0f, 5e-2f);
    }
}

TEST_F(TestHilbertTransform, fir_matches_fft_in_passband)
{
    static constexpr float f{ 8.0f };
    static constexpr std::size_t delay{ 15 };
    input.resize(N);
    for (std::size_t n{ 0 }; n < N; ++n)
        input[n] = std::cos(2.0f * std::numbers::pi_v<float> * f * static_cast<float>(n) / static_cast<float>(N));

    auto& a{ hilbert.Analytic(input) };

    typename infra::BoundedVector<float>::template WithMaxSize<N + delay> extended{};
    extended.resize(N + delay, 0.0f);
    for (std::size_t n{ 0 }; n < N; ++n)
        extended[n] = input[n];

    for (std::size_t n{ 10 }; n < 20; ++n)
    {
        for (std::size_t s{ 0 }; s < n + delay + 1; ++s)
        {
            float sample{ s < N ? input[s] : 0.0f };
            fir.Filter(sample);
        }
    }

    analysis::HilbertFir<float, 31> firFresh{};
    for (std::size_t s{ 0 }; s < delay; ++s)
        firFresh.Filter(input[s]);

    for (std::size_t n{ delay }; n < N - 5; ++n)
    {
        auto firOut{ firFresh.Filter(input[n]) };
        float fftEnv{ analysis::AnalyticSignalFft<float, N>::InstantaneousAmplitude(a[n - delay]) };
        float firEnv{ std::sqrt(firOut.Real() * firOut.Real() + firOut.Imaginary() * firOut.Imaginary()) };
        EXPECT_NEAR(firEnv, fftEnv, 1e-1f);
    }
}

TEST_F(TestHilbertTransform, dc_input_has_zero_hilbert)
{
    input.resize(N, 1.0f);

    auto& a{ hilbert.Analytic(input) };

    for (std::size_t n{ 1 }; n < N - 1; ++n)
        EXPECT_NEAR(a[n].Imaginary(), 0.0f, 1e-2f);
}
