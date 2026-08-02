#include "numerical/analysis/GoertzelAlgorithm.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gmock/gmock.h"
#include <array>
#include <cmath>
#include <numbers>

namespace
{
    class TestGoertzelAlgorithm : public ::testing::Test
    {
    protected:
        static constexpr std::size_t N = 16;
    };
}

TEST_F(TestGoertzelAlgorithm, not_ready_before_n_samples)
{
    analysis::GoertzelAlgorithm<float> goertzel{ std::size_t{ 2 }, N };

    for (std::size_t n = 0; n < N - 1; ++n)
        goertzel.Push(0.0f);

    EXPECT_FALSE(goertzel.Ready());
}

TEST_F(TestGoertzelAlgorithm, ready_after_n_samples)
{
    analysis::GoertzelAlgorithm<float> goertzel{ std::size_t{ 2 }, N };

    for (std::size_t n = 0; n < N; ++n)
        goertzel.Push(0.0f);

    EXPECT_TRUE(goertzel.Ready());
}

TEST_F(TestGoertzelAlgorithm, magnitude_matches_dft_bin_on_tone)
{
    constexpr std::size_t k{ 2 };
    analysis::GoertzelAlgorithm<float> goertzel{ k, N };

    for (std::size_t n = 0; n < N; ++n)
    {
        float sample{ std::cos(2.0f * std::numbers::pi_v<float> * static_cast<float>(k) * static_cast<float>(n) / static_cast<float>(N)) };
        goertzel.Push(sample);
    }

    EXPECT_NEAR(goertzel.Magnitude(), static_cast<float>(N) / 2.0f, 1e-2f);
}

TEST_F(TestGoertzelAlgorithm, magnitude_agrees_with_direct_dft_magnitude)
{
    constexpr std::size_t k{ 5 };
    analysis::GoertzelAlgorithm<float> goertzel{ k, N };

    std::array<float, N> signal{};
    for (std::size_t n = 0; n < N; ++n)
        signal[n] = std::cos(2.0f * std::numbers::pi_v<float> * 2.0f * static_cast<float>(n) / static_cast<float>(N)) + 0.5f;

    for (float s : signal)
        goertzel.Push(s);

    float refReal{ 0.0f };
    float refImag{ 0.0f };
    for (std::size_t n = 0; n < N; ++n)
    {
        float angle{ -2.0f * std::numbers::pi_v<float> * static_cast<float>(k) * static_cast<float>(n) / static_cast<float>(N) };
        refReal += signal[n] * std::cos(angle);
        refImag += signal[n] * std::sin(angle);
    }

    float refMag{ std::sqrt(refReal * refReal + refImag * refImag) };
    EXPECT_NEAR(goertzel.Magnitude(), refMag, 1e-3f);
}

TEST_F(TestGoertzelAlgorithm, result_matches_direct_dft_bin)
{
    constexpr std::size_t k{ 3 };
    analysis::GoertzelAlgorithm<float> goertzel{ k, N };

    std::array<float, N> signal{};
    for (std::size_t n = 0; n < N; ++n)
        signal[n] = std::cos(2.0f * std::numbers::pi_v<float> * static_cast<float>(k) * static_cast<float>(n) / static_cast<float>(N))
                    + 0.3f * std::sin(2.0f * std::numbers::pi_v<float> * static_cast<float>(k) * static_cast<float>(n) / static_cast<float>(N));

    for (float s : signal)
        goertzel.Push(s);

    float refReal{ 0.0f };
    float refImag{ 0.0f };
    for (std::size_t n = 0; n < N; ++n)
    {
        float angle{ -2.0f * std::numbers::pi_v<float> * static_cast<float>(k) * static_cast<float>(n) / static_cast<float>(N) };
        refReal += signal[n] * std::cos(angle);
        refImag += signal[n] * std::sin(angle);
    }

    math::Complex<float> result{ goertzel.Result() };
    EXPECT_NEAR(result.Real(), refReal, 1e-3f);
    EXPECT_NEAR(result.Imaginary(), refImag, 1e-3f);
}

TEST_F(TestGoertzelAlgorithm, magnitude_squared_equals_result_norm_squared)
{
    constexpr std::size_t k{ 2 };
    analysis::GoertzelAlgorithm<float> goertzel{ k, N };

    for (std::size_t n = 0; n < N; ++n)
    {
        float sample{ std::cos(2.0f * std::numbers::pi_v<float> * static_cast<float>(k) * static_cast<float>(n) / static_cast<float>(N)) };
        goertzel.Push(sample);
    }

    float mag{ goertzel.Magnitude() };
    math::Complex<float> result{ goertzel.Result() };
    float rSq{ result.Real() * result.Real() + result.Imaginary() * result.Imaginary() };
    EXPECT_NEAR(mag * mag, rSq, 1e-2f);
}

TEST_F(TestGoertzelAlgorithm, dc_bin_purely_real)
{
    constexpr float c{ 1.5f };
    analysis::GoertzelAlgorithm<float> goertzel{ std::size_t{ 0 }, N };

    for (std::size_t n = 0; n < N; ++n)
        goertzel.Push(c);

    math::Complex<float> result{ goertzel.Result() };
    EXPECT_NEAR(result.Real(), static_cast<float>(N) * c, 1e-3f);
    EXPECT_NEAR(result.Imaginary(), 0.0f, math::Tolerance<float>());
}

TEST_F(TestGoertzelAlgorithm, nyquist_bin_imaginary_is_zero)
{
    constexpr std::size_t kNyquist{ N / 2 };
    analysis::GoertzelAlgorithm<float> goertzel{ kNyquist, N };

    for (std::size_t n = 0; n < N; ++n)
    {
        float sample{ std::cos(std::numbers::pi_v<float> * static_cast<float>(n)) };
        goertzel.Push(sample);
    }

    math::Complex<float> result{ goertzel.Result() };
    EXPECT_NEAR(result.Imaginary(), 0.0f, math::Tolerance<float>());
    EXPECT_GT(std::abs(result.Real()), 1.0f);
}

TEST_F(TestGoertzelAlgorithm, zero_input_zero_output)
{
    analysis::GoertzelAlgorithm<float> goertzel{ std::size_t{ 3 }, N };

    for (std::size_t n = 0; n < N; ++n)
        goertzel.Push(0.0f);

    EXPECT_NEAR(goertzel.Magnitude(), 0.0f, math::Tolerance<float>());
    math::Complex<float> result{ goertzel.Result() };
    EXPECT_NEAR(result.Real(), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.Imaginary(), 0.0f, math::Tolerance<float>());
}

TEST_F(TestGoertzelAlgorithm, off_bin_tone_magnitude_below_half_peak)
{
    constexpr std::size_t kTarget{ 2 };
    constexpr std::size_t kOff{ 5 };
    analysis::GoertzelAlgorithm<float> goertzel{ kTarget, N };

    for (std::size_t n = 0; n < N; ++n)
    {
        float sample{ std::cos(2.0f * std::numbers::pi_v<float> * static_cast<float>(kOff) * static_cast<float>(n) / static_cast<float>(N)) };
        goertzel.Push(sample);
    }

    EXPECT_LT(goertzel.Magnitude(), static_cast<float>(N) / 4.0f);
}

TEST_F(TestGoertzelAlgorithm, frequency_hz_constructor_matches_bin_constructor)
{
    constexpr float sampleRate{ 1000.0f };
    constexpr float targetFreq{ 125.0f };
    constexpr std::size_t blockLen{ 8 };

    analysis::GoertzelAlgorithm<float> byBin{ std::size_t{ 1 }, blockLen };
    analysis::GoertzelAlgorithm<float> byHz{ targetFreq, sampleRate, blockLen };

    std::array<float, blockLen> signal{};
    for (std::size_t n = 0; n < blockLen; ++n)
        signal[n] = std::cos(2.0f * std::numbers::pi_v<float> * targetFreq * static_cast<float>(n) / sampleRate);

    for (float s : signal)
    {
        byBin.Push(s);
        byHz.Push(s);
    }

    EXPECT_NEAR(byBin.Magnitude(), byHz.Magnitude(), math::Tolerance<float>());
    math::Complex<float> rBin{ byBin.Result() };
    math::Complex<float> rHz{ byHz.Result() };
    EXPECT_NEAR(rBin.Real(), rHz.Real(), math::Tolerance<float>());
    EXPECT_NEAR(rBin.Imaginary(), rHz.Imaginary(), math::Tolerance<float>());
}

TEST_F(TestGoertzelAlgorithm, reset_restores_not_ready_and_yields_correct_magnitude)
{
    constexpr std::size_t k{ 2 };
    analysis::GoertzelAlgorithm<float> goertzel{ k, N };

    for (std::size_t n = 0; n < N; ++n)
        goertzel.Push(1.0f);

    goertzel.Reset();
    EXPECT_FALSE(goertzel.Ready());

    for (std::size_t n = 0; n < N; ++n)
    {
        float sample{ std::cos(2.0f * std::numbers::pi_v<float> * static_cast<float>(k) * static_cast<float>(n) / static_cast<float>(N)) };
        goertzel.Push(sample);
    }

    EXPECT_NEAR(goertzel.Magnitude(), static_cast<float>(N) / 2.0f, 1e-2f);
}

TEST_F(TestGoertzelAlgorithm, reset_output_matches_fresh_instance)
{
    constexpr std::size_t k{ 3 };
    analysis::GoertzelAlgorithm<float> goertzel{ k, N };

    std::array<float, N> signal{};
    for (std::size_t n = 0; n < N; ++n)
        signal[n] = std::cos(2.0f * std::numbers::pi_v<float> * static_cast<float>(k) * static_cast<float>(n) / static_cast<float>(N));

    for (float s : signal)
        goertzel.Push(s);

    goertzel.Reset();

    for (float s : signal)
        goertzel.Push(s);

    analysis::GoertzelAlgorithm<float> fresh{ k, N };
    for (float s : signal)
        fresh.Push(s);

    EXPECT_NEAR(goertzel.Magnitude(), fresh.Magnitude(), math::Tolerance<float>());
}

TEST_F(TestGoertzelAlgorithm, two_instances_do_not_interfere)
{
    constexpr std::size_t k1{ 2 };
    constexpr std::size_t k2{ 5 };
    analysis::GoertzelAlgorithm<float> g1{ k1, N };
    analysis::GoertzelAlgorithm<float> g2{ k2, N };

    for (std::size_t n = 0; n < N; ++n)
    {
        float s1{ std::cos(2.0f * std::numbers::pi_v<float> * static_cast<float>(k1) * static_cast<float>(n) / static_cast<float>(N)) };
        float s2{ std::cos(2.0f * std::numbers::pi_v<float> * static_cast<float>(k2) * static_cast<float>(n) / static_cast<float>(N)) };
        g1.Push(s1);
        g2.Push(s2);
    }

    EXPECT_NEAR(g1.Magnitude(), static_cast<float>(N) / 2.0f, 1e-2f);
    EXPECT_NEAR(g2.Magnitude(), static_cast<float>(N) / 2.0f, 1e-2f);
}

TEST_F(TestGoertzelAlgorithm, coefficient_formula_correct)
{
    constexpr std::size_t k{ 3 };
    float expected{ 2.0f * std::cos(2.0f * std::numbers::pi_v<float> * static_cast<float>(k) / static_cast<float>(N)) };
    EXPECT_NEAR(analysis::GoertzelAlgorithm<float>::Coefficient(k, N), expected, math::Tolerance<float>());
}
