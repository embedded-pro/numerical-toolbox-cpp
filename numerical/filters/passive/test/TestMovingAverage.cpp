#include "numerical/filters/passive/MovingAverage.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gtest/gtest.h"

namespace
{
    class TestMovingAverage
        : public ::testing::Test
    {
    public:
        filters::passive::MovingAverage<float, 4> ma{};
    };
}

TEST_F(TestMovingAverage, warmup_fills_window)
{
    constexpr float tol = 1e-6f;
    EXPECT_NEAR(ma.Filter(1.0f), 0.25f, tol);
    EXPECT_NEAR(ma.Filter(0.0f), 0.25f, tol);
    EXPECT_NEAR(ma.Filter(0.0f), 0.25f, tol);
    EXPECT_NEAR(ma.Filter(0.0f), 0.25f, tol);
}

TEST_F(TestMovingAverage, constant_input_unity_gain)
{
    constexpr float tol = 1e-6f;
    constexpr float c = 0.5f;
    for (int i = 0; i < 8; ++i)
        ma.Filter(c);
    EXPECT_NEAR(ma.Filter(c), c, tol);
}

TEST_F(TestMovingAverage, step_response_is_linear_ramp)
{
    constexpr float tol = 1e-6f;
    EXPECT_NEAR(ma.Filter(1.0f), 0.25f, tol);
    EXPECT_NEAR(ma.Filter(1.0f), 0.50f, tol);
    EXPECT_NEAR(ma.Filter(1.0f), 0.75f, tol);
    EXPECT_NEAR(ma.Filter(1.0f), 1.00f, tol);
    EXPECT_NEAR(ma.Filter(1.0f), 1.00f, tol);
}

TEST_F(TestMovingAverage, running_sum_matches_direct_average)
{
    constexpr float tol = 1e-6f;
    constexpr std::array<float, 8> seq{0.1f, 0.4f, 0.9f, 0.2f, 0.7f, 0.3f, 0.5f, 0.8f};
    std::array<float, 8> outputs{};
    for (std::size_t i = 0; i < seq.size(); ++i)
        outputs[i] = ma.Filter(seq[i]);

    for (std::size_t i = 3; i < seq.size(); ++i)
    {
        float brute = (seq[i] + seq[i - 1] + seq[i - 2] + seq[i - 3]) * 0.25f;
        EXPECT_NEAR(outputs[i], brute, tol);
    }
}

TEST_F(TestMovingAverage, impulse_leaves_after_N)
{
    constexpr float tol = 1e-6f;
    ma.Filter(1.0f);
    ma.Filter(0.0f);
    ma.Filter(0.0f);
    ma.Filter(0.0f);
    EXPECT_NEAR(ma.Filter(0.0f), 0.0f, tol);
    EXPECT_NEAR(ma.Filter(0.0f), 0.0f, tol);
}

TEST_F(TestMovingAverage, reset_clears_state)
{
    constexpr float tol = 1e-6f;
    ma.Filter(1.0f);
    ma.Filter(1.0f);
    ma.Reset();
    EXPECT_NEAR(ma.Filter(1.0f), 0.25f, tol);
}

TEST_F(TestMovingAverage, reset_with_value_prefills_window)
{
    constexpr float tol = 1e-6f;
    ma.Reset(2.0f);
    EXPECT_NEAR(ma.Filter(2.0f), 2.0f, tol);
}
