#include "numerical/estimators/offline/YuleWalker.hpp"
#include "numerical/math/Tolerance.hpp"
#include <gtest/gtest.h>

namespace
{
    class TestYuleWalkerFloat
        : public ::testing::Test
    {
    protected:
        static constexpr std::size_t Samples = 256;
        static constexpr std::size_t Order = 2;

        using SignalVector = math::Vector<float, Samples>;
        using PastVector = math::Vector<float, Order>;

        estimators::YuleWalker<float, Samples, Order> estimator;

        SignalVector MakeAr2Signal(float a1, float a2) const
        {
            SignalVector signal;
            signal[0] = 0.1f;
            signal[1] = 0.2f;
            for (std::size_t t = 2; t < Samples; ++t)
                signal[t] = a1 * signal[t - 1] + a2 * signal[t - 2];
            return signal;
        }

        SignalVector MakeNoisyAr2Signal(float a1, float a2) const
        {
            SignalVector signal;
            unsigned int seed = 42u;
            auto nextNoise = [&seed]()
            {
                seed = seed * 1103515245u + 12345u;
                return 0.01f * ((static_cast<float>((seed >> 16) & 0x7FFF) / 16384.0f) - 1.0f);
            };
            signal[0] = nextNoise();
            signal[1] = a1 * signal[0] + nextNoise();
            for (std::size_t t = 2; t < Samples; ++t)
                signal[t] = a1 * signal[t - 1] + a2 * signal[t - 2] + nextNoise();
            return signal;
        }

        SignalVector MakeConstantSignal(float value) const
        {
            SignalVector signal;
            for (std::size_t t = 0; t < Samples; ++t)
                signal[t] = value;
            return signal;
        }
    };
}

TEST_F(TestYuleWalkerFloat, fit_recovers_ar2_coefficients)
{
    auto signal = MakeNoisyAr2Signal(0.5f, -0.3f);

    estimator.Fit(signal);

    auto coeffs = estimator.Coefficients();
    EXPECT_NEAR(coeffs[0], 0.5f, 0.1f);
    EXPECT_NEAR(coeffs[1], -0.3f, 0.1f);
}

TEST_F(TestYuleWalkerFloat, predict_uses_coefficients)
{
    auto signal = MakeAr2Signal(0.5f, -0.3f);

    estimator.Fit(signal);

    PastVector past;
    past[0] = signal[Samples - 1];
    past[1] = signal[Samples - 2];

    float prediction = estimator.Predict(past);
    float expected = 0.5f * signal[Samples - 1] - 0.3f * signal[Samples - 2];
    EXPECT_NEAR(prediction, expected, 0.1f);
}

TEST_F(TestYuleWalkerFloat, noise_variance_is_nonnegative)
{
    auto signal = MakeNoisyAr2Signal(0.5f, -0.3f);

    estimator.Fit(signal);

    EXPECT_GE(estimator.NoiseVariance(), 0.0f);
}

TEST_F(TestYuleWalkerFloat, constant_signal_yields_near_zero_coefficients)
{
    auto signal = MakeConstantSignal(0.5f);

    estimator.Fit(signal);

    auto coeffs = estimator.Coefficients();
    EXPECT_NEAR(coeffs[0], 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(coeffs[1], 0.0f, math::Tolerance<float>());
}
