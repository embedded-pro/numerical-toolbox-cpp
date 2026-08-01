#include "numerical/math/GivensRotation.hpp"
#include "numerical/math/Tolerance.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    class GivensRotationTest : public ::testing::Test
    {
    };
}

TEST_F(GivensRotationTest, ZerosSecondComponent)
{
    auto g = math::ComputeGivens(3.0f, 4.0f);

    float x = 3.0f;
    float y = 4.0f;
    math::ApplyGivens(g, x, y);

    EXPECT_NEAR(x, 5.0f, math::Tolerance<float>());
    EXPECT_NEAR(y, 0.0f, math::Tolerance<float>());
}

TEST_F(GivensRotationTest, PreservesNorm)
{
    auto g = math::ComputeGivens(1.0f, 2.0f);

    float x = -3.0f;
    float y = 7.0f;
    float before = std::sqrt(x * x + y * y);
    math::ApplyGivens(g, x, y);

    EXPECT_NEAR(std::sqrt(x * x + y * y), before, math::Tolerance<float>());
}

TEST_F(GivensRotationTest, DegenerateInputIsIdentity)
{
    auto g = math::ComputeGivens(0.0f, 0.0f);

    EXPECT_NEAR(g.c, 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(g.s, 0.0f, math::Tolerance<float>());
}

TEST_F(GivensRotationTest, JacobiRotationAnnihilatesSymmetricOffDiagonal)
{
    float app = 4.0f;
    float aqq = 1.0f;
    float apq = 2.0f;

    auto g = math::ComputeJacobiRotation(app, aqq, apq);
    float offDiagonal = (g.c * g.c - g.s * g.s) * apq + g.c * g.s * (app - aqq);

    EXPECT_NEAR(offDiagonal, 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(g.c * g.c + g.s * g.s, 1.0f, math::Tolerance<float>());
}

TEST_F(GivensRotationTest, JacobiRotationZeroOffDiagonalIsIdentity)
{
    auto g = math::ComputeJacobiRotation(3.0f, 5.0f, 0.0f);

    EXPECT_NEAR(g.c, 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(g.s, 0.0f, math::Tolerance<float>());
}
