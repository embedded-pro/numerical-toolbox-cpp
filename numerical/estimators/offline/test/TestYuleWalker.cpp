#include "numerical/estimators/offline/YuleWalker.hpp"
#include "numerical/math/QNumber.hpp"
#include <gtest/gtest.h>

namespace
{
    class TestYuleWalkerFloat
        : public ::testing::Test
    {
    protected:
        static constexpr std::size_t Samples = 256;
        static constexpr std::size_t Order = 2;

        estimators::YuleWalker<float, Samples, Order> estimator;
    };
}

TEST_F(TestYuleWalkerFloat, fit_produces_nonzero_coefficients)
{
    math::Vector<float, TestYuleWalkerFloat::Samples> signal;
    signal[0] = 0.1f;
    signal[1] = 0.2f;
    for (std::size_t t = 2; t < TestYuleWalkerFloat::Samples; ++t)
        signal[t] = 0.5f * signal[t - 1] - 0.3f * signal[t - 2] + 0.01f * static_cast<float>(t % 7 - 3);

    estimator.Fit(signal);

    auto coeffs = estimator.Coefficients();
    EXPECT_NE(coeffs[0], 0.0f);
    EXPECT_NE(coeffs[1], 0.0f);
}

TEST_F(TestYuleWalkerFloat, fit_recovers_ar2_coefficients)
{
    math::Vector<float, TestYuleWalkerFloat::Samples> signal;

    unsigned int seed = 42u;
    auto nextNoise = [&seed]()
    {
        seed = seed * 1103515245u + 12345u;
        return 0.01f * ((static_cast<float>((seed >> 16) & 0x7FFF) / 16384.0f) - 1.0f);
    };

    signal[0] = nextNoise();
    signal[1] = 0.5f * signal[0] + nextNoise();
    for (std::size_t t = 2; t < TestYuleWalkerFloat::Samples; ++t)
        signal[t] = 0.5f * signal[t - 1] - 0.3f * signal[t - 2] + nextNoise();

    estimator.Fit(signal);

    auto coeffs = estimator.Coefficients();
    EXPECT_NEAR(coeffs[0], 0.5f, 0.1f);
    EXPECT_NEAR(coeffs[1], -0.3f, 0.1f);
}

TEST_F(TestYuleWalkerFloat, predict_uses_coefficients)
{
    math::Vector<float, TestYuleWalkerFloat::Samples> signal;
    signal[0] = 0.1f;
    signal[1] = 0.2f;
    for (std::size_t t = 2; t < TestYuleWalkerFloat::Samples; ++t)
        signal[t] = 0.5f * signal[t - 1] - 0.3f * signal[t - 2];

    estimator.Fit(signal);

    math::Vector<float, TestYuleWalkerFloat::Order> past;
    past[0] = signal[TestYuleWalkerFloat::Samples - 1];
    past[1] = signal[TestYuleWalkerFloat::Samples - 2];

    float prediction = estimator.Predict(past);

    float expected = 0.5f * signal[TestYuleWalkerFloat::Samples - 1] - 0.3f * signal[TestYuleWalkerFloat::Samples - 2];
    EXPECT_NEAR(prediction, expected, 0.1f);
}

TEST_F(TestYuleWalkerFloat, noise_variance_is_nonnegative)
{
    math::Vector<float, TestYuleWalkerFloat::Samples> signal;
    signal[0] = 0.1f;
    signal[1] = 0.2f;
    for (std::size_t t = 2; t < TestYuleWalkerFloat::Samples; ++t)
        signal[t] = 0.5f * signal[t - 1] - 0.3f * signal[t - 2] + 0.01f * static_cast<float>(t % 7 - 3);

    estimator.Fit(signal);

    EXPECT_GE(estimator.NoiseVariance(), 0.0f);
}

TEST_F(TestYuleWalkerFloat, constant_signal_yields_near_zero_coefficients)
{
    math::Vector<float, TestYuleWalkerFloat::Samples> signal;
    for (std::size_t t = 0; t < TestYuleWalkerFloat::Samples; ++t)
        signal[t] = 0.5f;

    estimator.Fit(signal);

    auto coeffs = estimator.Coefficients();
    EXPECT_NEAR(coeffs[0], 0.0f, 0.01f);
    EXPECT_NEAR(coeffs[1], 0.0f, 0.01f);
}
