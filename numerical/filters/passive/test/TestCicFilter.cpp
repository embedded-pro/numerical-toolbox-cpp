#include "numerical/filters/passive/CicFilter.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gtest/gtest.h"

namespace
{
    class TestCicDecimator
        : public ::testing::Test
    {
    public:
        filters::passive::CicDecimator<float, 2, 4, 1> cic{};
    };
}

TEST_F(TestCicDecimator, emits_one_per_R)
{
    int count{ 0 };
    for (int i = 0; i < 8; ++i)
    {
        auto result = cic.Filter(1.0f);
        if (result.valid)
            ++count;
    }
    EXPECT_EQ(count, 2);
}

TEST_F(TestCicDecimator, dc_gain_normalized)
{
    constexpr float c{ 0.5f };
    constexpr float tol{ 1e-5f };
    filters::passive::CicSample<float> result{};
    for (int i = 0; i < 16; ++i)
        result = cic.Filter(c);
    EXPECT_NEAR(result.value, c, tol);
}

TEST_F(TestCicDecimator, impulse_response_is_triangular)
{
    constexpr float tol{ 1e-5f };
    filters::passive::CicSample<float> first{};
    filters::passive::CicSample<float> second{};
    for (int i = 0; i < 8; ++i)
    {
        float inp = (i == 0) ? 1.0f : 0.0f;
        auto r = cic.Filter(inp);
        if (r.valid)
        {
            if (!first.valid)
                first = r;
            else if (!second.valid)
                second = r;
        }
    }
    EXPECT_NEAR(first.value, 0.25f, tol);
    EXPECT_NEAR(second.value, 0.0f, tol);
}

TEST_F(TestCicDecimator, silence_gives_zero)
{
    constexpr float tol{ 1e-5f };
    for (int i = 0; i < 8; ++i)
    {
        auto result = cic.Filter(0.0f);
        if (result.valid)
            EXPECT_NEAR(result.value, 0.0f, tol);
    }
}

TEST_F(TestCicDecimator, reset_clears_all_state)
{
    constexpr float tol{ 1e-5f };
    for (int i = 0; i < 4; ++i)
        cic.Filter(1.0f);
    cic.Reset();
    filters::passive::CicSample<float> first{};
    for (int i = 0; i < 4; ++i)
    {
        auto r = cic.Filter(1.0f);
        if (r.valid)
            first = r;
    }
    EXPECT_NEAR(first.value, 0.625f, tol);
}
