#include "numerical/filters/passive/MovingAverage.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gtest/gtest.h"
#include <array>
#include <cmath>
#include <random>

namespace
{
    class TestMovingAverage
        : public ::testing::Test
    {
    public:
        filters::passive::MovingAverage<float, 4> ma{};
    };

    class TestMovingAverageN1
        : public ::testing::Test
    {
    public:
        filters::passive::MovingAverage<float, 1> ma{};
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
    constexpr std::array<float, 8> seq{ 0.1f, 0.4f, 0.9f, 0.2f, 0.7f, 0.3f, 0.5f, 0.8f };
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

TEST_F(TestMovingAverage, constructor_nonzero_initial_reaches_steady_state_immediately)
{
    filters::passive::MovingAverage<float, 4> filter{ 3.0f };
    EXPECT_NEAR(filter.Filter(3.0f), 3.0f, 1e-6f);
}

TEST_F(TestMovingAverage, reset_with_value_flushed_by_n_different_inputs)
{
    constexpr float tol = 1e-6f;
    ma.Reset(2.0f);
    EXPECT_NEAR(ma.Filter(5.0f), 2.75f, tol);
    EXPECT_NEAR(ma.Filter(5.0f), 3.50f, tol);
    EXPECT_NEAR(ma.Filter(5.0f), 4.25f, tol);
    EXPECT_NEAR(ma.Filter(5.0f), 5.00f, tol);
    EXPECT_NEAR(ma.Filter(5.0f), 5.00f, tol);
}

TEST_F(TestMovingAverage, negative_inputs_step_response)
{
    constexpr float tol = 1e-6f;
    EXPECT_NEAR(ma.Filter(-1.0f), -0.25f, tol);
    EXPECT_NEAR(ma.Filter(-1.0f), -0.50f, tol);
    EXPECT_NEAR(ma.Filter(-1.0f), -0.75f, tol);
    EXPECT_NEAR(ma.Filter(-1.0f), -1.00f, tol);
    EXPECT_NEAR(ma.Filter(-1.0f), -1.00f, tol);
}

TEST_F(TestMovingAverage, nyquist_attenuation_is_zero)
{
    for (int i = 0; i < 100; ++i)
        ma.Filter((i % 2 == 0) ? 1.0f : -1.0f);

    float sumSqOut{};
    for (int i = 0; i < 8; ++i)
    {
        const float y = ma.Filter((i % 2 == 0) ? 1.0f : -1.0f);
        sumSqOut += y * y;
    }

    EXPECT_NEAR(sumSqOut, 0.0f, math::Tolerance<float>());
}

TEST_F(TestMovingAverage, reset_restores_fresh_instance_behaviour)
{
    ma.Filter(7.0f);
    ma.Filter(3.0f);
    ma.Reset();

    filters::passive::MovingAverage<float, 4> fresh{};
    constexpr std::array<float, 6> seq{ 0.1f, 0.4f, 0.9f, 0.2f, 0.7f, 0.3f };
    for (const float x : seq)
        EXPECT_FLOAT_EQ(ma.Filter(x), fresh.Filter(x));
}

TEST_F(TestMovingAverage, determinism_two_instances_identical_output)
{
    filters::passive::MovingAverage<float, 4> f1{};
    filters::passive::MovingAverage<float, 4> f2{};

    constexpr std::array<float, 8> seq{ 0.1f, 0.4f, 0.9f, 0.2f, 0.7f, 0.3f, 0.5f, 0.8f };
    for (const float x : seq)
        EXPECT_FLOAT_EQ(f1.Filter(x), f2.Filter(x));
}

TEST_F(TestMovingAverage, linearity_superposition)
{
    constexpr float a = 2.0f;
    constexpr float b = -0.5f;

    std::mt19937 rng{ 42u };
    std::uniform_real_distribution<float> dist{ -1.0f, 1.0f };

    constexpr int n = 16;
    std::array<float, n> xSeq{};
    std::array<float, n> ySeq{};
    for (int i = 0; i < n; ++i)
    {
        xSeq[i] = dist(rng);
        ySeq[i] = dist(rng);
    }

    filters::passive::MovingAverage<float, 4> fxy{};
    filters::passive::MovingAverage<float, 4> fx{};
    filters::passive::MovingAverage<float, 4> fy{};

    for (int i = 0; i < n; ++i)
    {
        const float combined = fxy.Filter(a * xSeq[i] + b * ySeq[i]);
        const float scaled = a * fx.Filter(xSeq[i]) + b * fy.Filter(ySeq[i]);
        EXPECT_NEAR(combined, scaled, math::Tolerance<float>());
    }
}

TEST_F(TestMovingAverage, bibo_stability_bounded_input_bounded_output)
{
    std::mt19937 rng{ 7u };
    constexpr float bound = 1.0f;
    std::uniform_real_distribution<float> dist{ -bound, bound };

    for (int i = 0; i < 10000; ++i)
    {
        const float out = ma.Filter(dist(rng));
        EXPECT_TRUE(std::isfinite(out));
        EXPECT_LE(std::abs(out), bound + math::Tolerance<float>());
    }
}

TEST_F(TestMovingAverageN1, n1_is_passthrough)
{
    constexpr std::array<float, 4> inputs{ 0.3f, -0.7f, 1.0f, 0.0f };
    for (const float x : inputs)
        EXPECT_NEAR(ma.Filter(x), x, 1e-6f);
}

TEST_F(TestMovingAverageN1, n1_reset_clears_and_is_passthrough)
{
    ma.Filter(5.0f);
    ma.Reset();
    constexpr std::array<float, 3> inputs{ 0.1f, -0.2f, 0.9f };
    for (const float x : inputs)
        EXPECT_NEAR(ma.Filter(x), x, 1e-6f);
}
