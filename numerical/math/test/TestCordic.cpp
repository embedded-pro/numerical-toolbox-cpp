#include "numerical/math/Cordic.hpp"
#include "numerical/math/Tolerance.hpp"
#include <array>
#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    class TestCordic
        : public ::testing::Test
    {
    protected:
        math::Cordic<float, 16> cordic{};
    };
}

TEST_F(TestCordic, SinCosZero)
{
    auto result = cordic.SineCosine(0.0f);
    EXPECT_NEAR(result.sin, 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.cos, 1.0f, math::Tolerance<float>());
}

TEST_F(TestCordic, SinCosQuarterPi)
{
    auto result = cordic.SineCosine(std::numbers::pi_v<float> / 4.0f);
    EXPECT_NEAR(result.sin, 0.7071f, math::Tolerance<float>());
    EXPECT_NEAR(result.cos, 0.7071f, math::Tolerance<float>());
}

TEST_F(TestCordic, SinCosMatchesStdOverSweep)
{
    const float tolerance{ 1.0f / static_cast<float>(1 << 15) };
    std::array<float, 9> angles{ -1.5f, -1.0f, -0.5f, -0.25f, 0.0f, 0.25f, 0.5f, 1.0f, 1.5f };
    for (float angle : angles)
    {
        auto result = cordic.SineCosine(angle);
        EXPECT_NEAR(result.sin, std::sin(angle), tolerance);
        EXPECT_NEAR(result.cos, std::cos(angle), tolerance);
    }
}

TEST_F(TestCordic, PythagoreanIdentity)
{
    std::array<float, 7> angles{ -1.5f, -1.0f, -0.5f, 0.0f, 0.5f, 1.0f, 1.5f };
    for (float angle : angles)
    {
        auto result = cordic.SineCosine(angle);
        float identity{ result.sin * result.sin + result.cos * result.cos };
        EXPECT_NEAR(identity, 1.0f, math::Tolerance<float>());
    }
}

TEST_F(TestCordic, Atan2AllQuadrants)
{
    const float tol{ math::Tolerance<float>() };
    EXPECT_NEAR(cordic.Arctangent2(1.0f, 1.0f), std::atan2(1.0f, 1.0f), tol);
    EXPECT_NEAR(cordic.Arctangent2(1.0f, -1.0f), std::atan2(1.0f, -1.0f), tol);
    EXPECT_NEAR(cordic.Arctangent2(-1.0f, -1.0f), std::atan2(-1.0f, -1.0f), tol);
    EXPECT_NEAR(cordic.Arctangent2(-1.0f, 1.0f), std::atan2(-1.0f, 1.0f), tol);
}

TEST_F(TestCordic, Atan2Axes)
{
    const float pi{ std::numbers::pi_v<float> };
    const float tol{ math::Tolerance<float>() };
    EXPECT_NEAR(cordic.Arctangent2(0.0f, 1.0f), 0.0f, tol);
    EXPECT_NEAR(cordic.Arctangent2(1.0f, 0.0f), pi / 2.0f, tol);
    EXPECT_NEAR(cordic.Arctangent2(0.0f, -1.0f), pi, tol);
}

TEST_F(TestCordic, MagnitudeMatchesHypot)
{
    const float tol{ math::Tolerance<float>() };
    EXPECT_NEAR(cordic.Magnitude(0.6f, 0.8f), 1.0f, tol);
    EXPECT_NEAR(cordic.Magnitude(0.3f, 0.4f), 0.5f, tol);
    EXPECT_NEAR(cordic.Magnitude(0.0f, 0.5f), 0.5f, tol);
}

TEST_F(TestCordic, RotateVector)
{
    const float pi{ std::numbers::pi_v<float> };
    std::array<float, 2> v{ 1.0f, 0.0f };
    auto result = cordic.Rotate(v, pi / 2.0f);
    EXPECT_NEAR(result[0], 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(result[1], 1.0f, math::Tolerance<float>());
}

TEST_F(TestCordic, SinCosAboveHalfPi)
{
    const float angle{ 2.0f };
    const float tol{ 1.0f / static_cast<float>(1 << 15) };
    auto result = cordic.SineCosine(angle);
    EXPECT_NEAR(result.sin, std::sin(angle), tol);
    EXPECT_NEAR(result.cos, std::cos(angle), tol);
}

TEST_F(TestCordic, SinCosBelowNegHalfPi)
{
    const float angle{ -2.0f };
    const float tol{ 1.0f / static_cast<float>(1 << 15) };
    auto result = cordic.SineCosine(angle);
    EXPECT_NEAR(result.sin, std::sin(angle), tol);
    EXPECT_NEAR(result.cos, std::cos(angle), tol);
}

TEST_F(TestCordic, AccuracyScalesWithIterations)
{
    math::Cordic<float, 8> cordic8{};
    const float angle{ 0.7f };
    float ref{ std::sin(angle) };
    float err8{ std::abs(cordic8.SineCosine(angle).sin - ref) };
    float err16{ std::abs(cordic.SineCosine(angle).sin - ref) };
    EXPECT_LT(err16, err8);
}
