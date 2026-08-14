#include "numerical/math/QNumber.hpp"
#include "numerical/math/Tolerance.hpp"
#include <gtest/gtest.h>

namespace
{
    class TestTolerance : public ::testing::Test
    {};
}

TEST_F(TestTolerance, tolerance_for_float_returns_1e_minus_3)
{
    EXPECT_FLOAT_EQ(math::Tolerance<float>(), 1e-3f);
}

TEST_F(TestTolerance, tolerance_for_q31_returns_1e_minus_3)
{
    EXPECT_FLOAT_EQ(math::Tolerance<math::Q31>(), 1e-3f);
}

TEST_F(TestTolerance, tolerance_for_q15_returns_1e_minus_3)
{
    EXPECT_FLOAT_EQ(math::Tolerance<math::Q15>(), 1e-3f);
}

TEST_F(TestTolerance, tolerance_is_positive)
{
    EXPECT_GT(math::Tolerance<float>(), 0.0f);
}

TEST_F(TestTolerance, tolerance_is_less_than_one)
{
    EXPECT_LT(math::Tolerance<float>(), 1.0f);
}
