#include "numerical/math/Math.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    class TestMath : public ::testing::Test
    {
    protected:
        static constexpr float kEps = 1e-5f;
        static constexpr float kPi  = std::numbers::pi_v<float>;
        static constexpr float kPi2 = std::numbers::pi_v<float> / 2.0f;
        static constexpr float kPi4 = std::numbers::pi_v<float> / 4.0f;
    };
}

TEST_F(TestMath, Abs_positive_and_negative)
{
    EXPECT_FLOAT_EQ(math::Abs(2.0f), 2.0f);
    EXPECT_FLOAT_EQ(math::Abs(-3.5f), 3.5f);
}

TEST_F(TestMath, Sqrt_known_value)
{
    EXPECT_NEAR(math::Sqrt(4.0f), 2.0f, kEps);
}

TEST_F(TestMath, Sin_known_values)
{
    EXPECT_NEAR(math::Sin(0.0f), 0.0f, kEps);
    EXPECT_NEAR(math::Sin(kPi2), 1.0f, kEps);
}

TEST_F(TestMath, Cos_known_values)
{
    EXPECT_NEAR(math::Cos(0.0f), 1.0f, kEps);
    EXPECT_NEAR(math::Cos(kPi), -1.0f, kEps);
}

TEST_F(TestMath, Tan_known_value)
{
    EXPECT_NEAR(math::Tan(0.0f), 0.0f, kEps);
    EXPECT_NEAR(math::Tan(kPi4), 1.0f, 1e-4f);
}

TEST_F(TestMath, Asin_known_value)
{
    EXPECT_NEAR(math::Asin(0.0f), 0.0f, kEps);
    EXPECT_NEAR(math::Asin(1.0f), kPi2, kEps);
}

TEST_F(TestMath, Acos_known_value)
{
    EXPECT_NEAR(math::Acos(1.0f), 0.0f, kEps);
    EXPECT_NEAR(math::Acos(0.0f), kPi2, kEps);
}

TEST_F(TestMath, Atan_known_value)
{
    EXPECT_NEAR(math::Atan(0.0f), 0.0f, kEps);
    EXPECT_NEAR(math::Atan(1.0f), kPi4, kEps);
}

TEST_F(TestMath, Atan2_known_value)
{
    EXPECT_NEAR(math::Atan2(1.0f, 1.0f), kPi4, kEps);
    EXPECT_NEAR(math::Atan2(0.0f, 1.0f), 0.0f, kEps);
}

TEST_F(TestMath, Exp_known_value)
{
    EXPECT_NEAR(math::Exp(0.0f), 1.0f, kEps);
    EXPECT_NEAR(math::Exp(1.0f), std::exp(1.0f), kEps);
}

TEST_F(TestMath, Log_known_value)
{
    EXPECT_NEAR(math::Log(1.0f), 0.0f, kEps);
    EXPECT_NEAR(math::Log(std::exp(1.0f)), 1.0f, kEps);
}

TEST_F(TestMath, Log10_known_value)
{
    EXPECT_NEAR(math::Log10(1.0f), 0.0f, kEps);
    EXPECT_NEAR(math::Log10(100.0f), 2.0f, kEps);
}

TEST_F(TestMath, Log2_known_value)
{
    EXPECT_NEAR(math::Log2(1.0f), 0.0f, kEps);
    EXPECT_NEAR(math::Log2(8.0f), 3.0f, kEps);
}

TEST_F(TestMath, Pow_known_value)
{
    EXPECT_NEAR(math::Pow(2.0f, 3.0f), 8.0f, kEps);
    EXPECT_NEAR(math::Pow(4.0f, 0.5f), 2.0f, kEps);
}

TEST_F(TestMath, Sinh_known_value)
{
    EXPECT_NEAR(math::Sinh(0.0f), 0.0f, kEps);
    EXPECT_NEAR(math::Sinh(1.0f), std::sinh(1.0f), kEps);
}

TEST_F(TestMath, Cosh_known_value)
{
    EXPECT_NEAR(math::Cosh(0.0f), 1.0f, kEps);
    EXPECT_NEAR(math::Cosh(1.0f), std::cosh(1.0f), kEps);
}

TEST_F(TestMath, Tanh_known_value)
{
    EXPECT_NEAR(math::Tanh(0.0f), 0.0f, kEps);
    EXPECT_NEAR(math::Tanh(1.0f), std::tanh(1.0f), kEps);
}

TEST_F(TestMath, Hypot_known_value)
{
    EXPECT_NEAR(math::Hypot(3.0f, 4.0f), 5.0f, kEps);
}

TEST_F(TestMath, Copysign_transfers_sign)
{
    EXPECT_FLOAT_EQ(math::Copysign(3.0f, -1.0f), -3.0f);
    EXPECT_FLOAT_EQ(math::Copysign(-3.0f, 1.0f), 3.0f);
}

TEST_F(TestMath, Fmod_known_value)
{
    EXPECT_NEAR(math::Fmod(5.5f, 2.0f), 1.5f, kEps);
    EXPECT_NEAR(math::Fmod(7.0f, 3.0f), 1.0f, kEps);
}

TEST_F(TestMath, Ceil_rounds_up)
{
    EXPECT_FLOAT_EQ(math::Ceil(1.2f), 2.0f);
    EXPECT_FLOAT_EQ(math::Ceil(-1.2f), -1.0f);
}

TEST_F(TestMath, Floor_rounds_down)
{
    EXPECT_FLOAT_EQ(math::Floor(1.9f), 1.0f);
    EXPECT_FLOAT_EQ(math::Floor(-1.2f), -2.0f);
}

TEST_F(TestMath, Round_rounds_to_nearest)
{
    EXPECT_FLOAT_EQ(math::Round(1.5f), 2.0f);
    EXPECT_FLOAT_EQ(math::Round(1.4f), 1.0f);
}

TEST_F(TestMath, Erfc_known_value)
{
    EXPECT_NEAR(math::Erfc(0.0f), 1.0f, kEps);
    EXPECT_NEAR(math::Erfc(1.0f), std::erfc(1.0f), kEps);
}
