#include "numerical/filters/passive/MedianFilter.hpp"
#include "gtest/gtest.h"
#include <array>
#include <random>

namespace
{
    class TestMedianFilter
        : public ::testing::Test
    {
    protected:
        filters::passive::MedianFilter<float, 3> med{};
    };

    class TestMedianFilterN5
        : public ::testing::Test
    {
    protected:
        filters::passive::MedianFilter<float, 5> med{};
    };

    class TestMedianFilterInit
        : public ::testing::Test
    {
    protected:
        filters::passive::MedianFilter<float, 3> med{ 5.0f };
    };
}

TEST_F(TestMedianFilter, rejects_single_impulse)
{
    med.Filter(0.1f);
    med.Filter(0.1f);
    float y2 = med.Filter(0.9f);
    float y3 = med.Filter(0.1f);
    float y4 = med.Filter(0.1f);
    EXPECT_NEAR(y2, 0.1f, 1e-6f);
    EXPECT_NEAR(y3, 0.1f, 1e-6f);
    EXPECT_NEAR(y4, 0.1f, 1e-6f);
}

TEST_F(TestMedianFilter, preserves_step_edge)
{
    med.Filter(0.0f);
    med.Filter(0.0f);
    float y2 = med.Filter(1.0f);
    float y3 = med.Filter(1.0f);
    float y4 = med.Filter(1.0f);
    EXPECT_NEAR(y2, 0.0f, 1e-6f);
    EXPECT_NEAR(y3, 1.0f, 1e-6f);
    EXPECT_NEAR(y4, 1.0f, 1e-6f);
}

TEST_F(TestMedianFilter, constant_input_passthrough)
{
    constexpr float c = 0.7f;
    med.Filter(c);
    med.Filter(c);
    EXPECT_NEAR(med.Filter(c), c, 1e-6f);
    EXPECT_NEAR(med.Filter(c), c, 1e-6f);
}

TEST_F(TestMedianFilter, sorted_middle_is_returned)
{
    med.Filter(0.3f);
    med.Filter(0.1f);
    float result = med.Filter(0.2f);
    EXPECT_NEAR(result, 0.2f, 1e-6f);
}

TEST_F(TestMedianFilter, majority_spike_passes)
{
    med.Filter(0.0f);
    float y1 = med.Filter(0.9f);
    float y2 = med.Filter(0.9f);
    float y3 = med.Filter(0.0f);
    EXPECT_NEAR(y1, 0.0f, 1e-6f);
    EXPECT_NEAR(y2, 0.9f, 1e-6f);
    EXPECT_NEAR(y3, 0.9f, 1e-6f);
}

TEST_F(TestMedianFilter, reset_clears_window)
{
    med.Filter(1.0f);
    med.Filter(1.0f);
    med.Filter(1.0f);
    med.Reset(0.0f);
    EXPECT_NEAR(med.Filter(0.0f), 0.0f, 1e-6f);
    EXPECT_NEAR(med.Filter(0.0f), 0.0f, 1e-6f);
}

TEST_F(TestMedianFilter, negative_inputs_sorted_correctly)
{
    med.Filter(-0.3f);
    med.Filter(-0.1f);
    float result = med.Filter(-0.2f);
    EXPECT_NEAR(result, -0.2f, 1e-6f);
}

TEST_F(TestMedianFilter, negative_impulse_rejected)
{
    med.Filter(0.0f);
    med.Filter(0.0f);
    float y = med.Filter(-1.0f);
    EXPECT_NEAR(y, 0.0f, 1e-6f);
}

TEST_F(TestMedianFilter, consecutive_resets_restore_initial_state)
{
    med.Filter(1.0f);
    med.Filter(2.0f);
    med.Reset(3.0f);
    med.Reset(0.0f);
    EXPECT_NEAR(med.Filter(0.0f), 0.0f, 1e-6f);
    EXPECT_NEAR(med.Filter(0.0f), 0.0f, 1e-6f);
}

TEST_F(TestMedianFilter, determinism_same_sequence_yields_identical_output)
{
    filters::passive::MedianFilter<float, 3> med2{};
    constexpr std::array<float, 6> seq{ 0.1f, 0.5f, 0.3f, 0.8f, 0.2f, 0.6f };
    std::array<float, 6> out1{};
    std::array<float, 6> out2{};
    for (std::size_t i = 0; i < seq.size(); ++i)
    {
        out1[i] = med.Filter(seq[i]);
        out2[i] = med2.Filter(seq[i]);
    }
    for (std::size_t i = 0; i < seq.size(); ++i)
        EXPECT_FLOAT_EQ(out1[i], out2[i]);
}

TEST_F(TestMedianFilter, reset_then_run_matches_fresh_instance)
{
    med.Filter(10.0f);
    med.Filter(20.0f);
    med.Filter(30.0f);
    med.Reset();

    filters::passive::MedianFilter<float, 3> fresh{};
    constexpr std::array<float, 4> seq{ 1.0f, 2.0f, 3.0f, 4.0f };
    for (std::size_t i = 0; i < seq.size(); ++i)
        EXPECT_FLOAT_EQ(med.Filter(seq[i]), fresh.Filter(seq[i]));
}

TEST_F(TestMedianFilter, output_always_bounded_by_input_range)
{
    std::mt19937 rng{ 42u };
    std::uniform_real_distribution<float> dist{ -100.0f, 100.0f };
    float lo = -100.0f;
    float hi = 100.0f;
    for (int i = 0; i < 64; ++i)
    {
        float y = med.Filter(dist(rng));
        EXPECT_GE(y, lo);
        EXPECT_LE(y, hi);
    }
}

TEST_F(TestMedianFilter, output_is_order_statistic_of_window)
{
    med.Filter(0.5f);
    med.Filter(0.1f);
    float y = med.Filter(0.9f);
    EXPECT_NEAR(y, 0.5f, 1e-6f);
}

TEST_F(TestMedianFilterInit, constructor_initial_value_fills_window)
{
    EXPECT_NEAR(med.Filter(5.0f), 5.0f, 1e-6f);
}

TEST_F(TestMedianFilterInit, constructor_initial_value_suppresses_single_outlier)
{
    float y = med.Filter(100.0f);
    EXPECT_NEAR(y, 5.0f, 1e-6f);
}

TEST_F(TestMedianFilterN5, rejects_single_impulse_n5)
{
    med.Filter(1.0f);
    med.Filter(1.0f);
    med.Filter(1.0f);
    med.Filter(1.0f);
    float y = med.Filter(99.0f);
    EXPECT_NEAR(y, 1.0f, 1e-6f);
}

TEST_F(TestMedianFilterN5, constant_input_passthrough_n5)
{
    constexpr float c = 3.14f;
    med.Filter(c);
    med.Filter(c);
    med.Filter(c);
    med.Filter(c);
    EXPECT_NEAR(med.Filter(c), c, 1e-6f);
}

TEST_F(TestMedianFilterN5, sorted_middle_of_five)
{
    med.Filter(5.0f);
    med.Filter(1.0f);
    med.Filter(4.0f);
    med.Filter(2.0f);
    float result = med.Filter(3.0f);
    EXPECT_NEAR(result, 3.0f, 1e-6f);
}

TEST_F(TestMedianFilterN5, step_edge_propagates_after_majority)
{
    med.Filter(0.0f);
    med.Filter(0.0f);
    med.Filter(0.0f);
    float y3 = med.Filter(1.0f);
    float y4 = med.Filter(1.0f);
    float y5 = med.Filter(1.0f);
    EXPECT_NEAR(y3, 0.0f, 1e-6f);
    EXPECT_NEAR(y4, 0.0f, 1e-6f);
    EXPECT_NEAR(y5, 1.0f, 1e-6f);
}

TEST_F(TestMedianFilterN5, reset_restores_state_n5)
{
    med.Filter(9.0f);
    med.Filter(9.0f);
    med.Filter(9.0f);
    med.Filter(9.0f);
    med.Filter(9.0f);
    med.Reset(2.0f);
    EXPECT_NEAR(med.Filter(2.0f), 2.0f, 1e-6f);
}
