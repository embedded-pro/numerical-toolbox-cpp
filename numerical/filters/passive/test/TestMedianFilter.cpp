#include "numerical/filters/passive/MedianFilter.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gtest/gtest.h"

namespace
{
    class TestMedianFilter
        : public ::testing::Test
    {
    public:
        filters::passive::MedianFilter<float, 3> med{};
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
