#include "numerical/filters/passive/ExponentialMovingAverage.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gtest/gtest.h"

namespace
{
    class TestExponentialMovingAverage
        : public ::testing::Test
    {
    public:
        filters::passive::ExponentialMovingAverage<float> ema{ 0.5f };
    };
}

TEST_F(TestExponentialMovingAverage, impulse_decays_geometrically)
{
    const float inputs[]{ 1.0f, 0.0f, 0.0f, 0.0f };
    const float expected[]{ 0.5f, 0.25f, 0.125f, 0.0625f };

    for (int i = 0; i < 4; ++i)
        EXPECT_NEAR(ema.Filter(inputs[i]), expected[i], 1e-6f);
}

TEST_F(TestExponentialMovingAverage, constant_input_converges_to_dc)
{
    filters::passive::ExponentialMovingAverage<float> filter{ 0.25f };
    const float c{ 0.8f };

    float output{};
    for (int i = 0; i < 50; ++i)
        output = filter.Filter(c);

    EXPECT_NEAR(output, c, math::Tolerance<float>());
}

TEST_F(TestExponentialMovingAverage, alpha_one_is_passthrough)
{
    filters::passive::ExponentialMovingAverage<float> filter{ 1.0f };
    const float inputs[]{ 0.3f, -0.7f, 1.0f, 0.0f };

    for (const float x : inputs)
        EXPECT_NEAR(filter.Filter(x), x, 1e-6f);
}

TEST_F(TestExponentialMovingAverage, step_response_matches_time_constant)
{
    filters::passive::ExponentialMovingAverage<float> filter{ 0.1f };

    float output{};
    for (int i = 0; i < 10; ++i)
        output = filter.Filter(1.0f);

    EXPECT_NEAR(output, 0.6513f, 1e-3f);
}

TEST_F(TestExponentialMovingAverage, reset_clears_state)
{
    ema.Filter(1.0f);
    ema.Filter(1.0f);
    ema.Reset(0.0f);

    const float nextInput{ 0.6f };
    EXPECT_NEAR(ema.Filter(nextInput), 0.5f * nextInput, 1e-6f);
}

TEST_F(TestExponentialMovingAverage, disabled_passes_through)
{
    ema.Filter(1.0f);
    ema.Disable();

    const float input{ 0.4f };
    EXPECT_NEAR(ema.Filter(input), input, 1e-6f);
    EXPECT_NEAR(ema.Filter(0.9f), 0.9f, 1e-6f);
}
