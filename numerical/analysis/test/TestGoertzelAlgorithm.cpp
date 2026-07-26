#include "numerical/analysis/GoertzelAlgorithm.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gmock/gmock.h"
#include <cmath>
#include <numbers>
#include <array>

namespace
{
    class TestGoertzelAlgorithm : public ::testing::Test
    {
    public:
        static constexpr std::size_t N = 16;
    };
}

TEST_F(TestGoertzelAlgorithm, detects_matching_tone)
{
    analysis::GoertzelAlgorithm<float> goertzel{ std::size_t{ 2 }, N };

    for (std::size_t n = 0; n < N; ++n)
    {
        float sample{ std::cos(2.0f * std::numbers::pi_v<float> * 2.0f * static_cast<float>(n) / static_cast<float>(N)) };
        goertzel.Push(sample);
    }

    EXPECT_TRUE(goertzel.Ready());
    EXPECT_NEAR(goertzel.Magnitude(), static_cast<float>(N) / 2.0f, 1e-2f);
}

TEST_F(TestGoertzelAlgorithm, rejects_off_bin_tone)
{
    analysis::GoertzelAlgorithm<float> goertzel{ std::size_t{ 2 }, N };

    for (std::size_t n = 0; n < N; ++n)
    {
        float sample{ std::cos(2.0f * std::numbers::pi_v<float> * 5.0f * static_cast<float>(n) / static_cast<float>(N)) };
        goertzel.Push(sample);
    }

    EXPECT_LT(goertzel.Magnitude(), 1.0f);
}

TEST_F(TestGoertzelAlgorithm, magnitude_matches_dft_bin)
{
    constexpr std::size_t k{ 3 };
    analysis::GoertzelAlgorithm<float> goertzel{ k, N };

    std::array<float, N> signal{};
    for (std::size_t n = 0; n < N; ++n)
        signal[n] = std::cos(2.0f * std::numbers::pi_v<float> * static_cast<float>(n) / static_cast<float>(N)) + 0.5f;

    for (std::size_t n = 0; n < N; ++n)
        goertzel.Push(signal[n]);

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

TEST_F(TestGoertzelAlgorithm, coefficient_formula_correct)
{
    constexpr std::size_t k{ 3 };
    float expected{ 2.0f * std::cos(2.0f * std::numbers::pi_v<float> * static_cast<float>(k) / static_cast<float>(N)) };
    EXPECT_NEAR(analysis::GoertzelAlgorithm<float>::Coefficient(k, N), expected, math::Tolerance<float>());
}

TEST_F(TestGoertzelAlgorithm, dc_bin_sums_input)
{
    constexpr float c{ 1.5f };
    analysis::GoertzelAlgorithm<float> goertzel{ std::size_t{ 0 }, N };

    for (std::size_t n = 0; n < N; ++n)
        goertzel.Push(c);

    math::Complex<float> result{ goertzel.Result() };
    EXPECT_NEAR(result.Real(), static_cast<float>(N) * c, 1e-3f);
}

TEST_F(TestGoertzelAlgorithm, magnitude_squared_matches_result)
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

TEST_F(TestGoertzelAlgorithm, reset_restarts_block)
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

TEST_F(TestGoertzelAlgorithm, not_ready_before_n_samples)
{
    analysis::GoertzelAlgorithm<float> goertzel{ std::size_t{ 2 }, N };

    for (std::size_t n = 0; n < N - 1; ++n)
        goertzel.Push(0.0f);

    EXPECT_FALSE(goertzel.Ready());
}
