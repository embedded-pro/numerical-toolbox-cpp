#include "numerical/math/GivensRotation.hpp"
#include "numerical/math/Tolerance.hpp"
#include <array>
#include <cmath>
#include <gtest/gtest.h>
#include <random>

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

TEST_F(GivensRotationTest, ComputeGivensUnitarity)
{
    auto g = math::ComputeGivens(5.0f, 12.0f);

    EXPECT_NEAR(g.c * g.c + g.s * g.s, 1.0f, math::Tolerance<float>());
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

TEST_F(GivensRotationTest, SubthresholdInputIsIdentity)
{
    auto g = math::ComputeGivens(1e-31f, 1e-31f);

    EXPECT_NEAR(g.c, 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(g.s, 0.0f, math::Tolerance<float>());
}

TEST_F(GivensRotationTest, ApplyGivensOrthogonalBasisVectors)
{
    auto g = math::ComputeGivens(3.0f, 4.0f);

    float x0 = 1.0f, y0 = 0.0f;
    float x1 = 0.0f, y1 = 1.0f;
    math::ApplyGivens(g, x0, y0);
    math::ApplyGivens(g, x1, y1);

    float dot = x0 * x1 + y0 * y1;
    EXPECT_NEAR(dot, 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(x0 * x0 + y0 * y0, 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(x1 * x1 + y1 * y1, 1.0f, math::Tolerance<float>());
}

TEST_F(GivensRotationTest, JacobiRotationPositiveTheta)
{
    float app = 1.0f, aqq = 4.0f, apq = 2.0f;
    auto g = math::ComputeJacobiRotation(app, aqq, apq);

    float ref_c = 1.0f / std::sqrt(1.25f);
    float ref_s = 0.5f / std::sqrt(1.25f);

    EXPECT_NEAR(g.c, ref_c, math::Tolerance<float>());
    EXPECT_NEAR(g.s, ref_s, math::Tolerance<float>());
}

TEST_F(GivensRotationTest, JacobiRotationNegativeTheta)
{
    float app = 4.0f, aqq = 1.0f, apq = 2.0f;
    auto g = math::ComputeJacobiRotation(app, aqq, apq);

    float ref_c = 1.0f / std::sqrt(1.25f);
    float ref_s = -0.5f / std::sqrt(1.25f);

    EXPECT_NEAR(g.c, ref_c, math::Tolerance<float>());
    EXPECT_NEAR(g.s, ref_s, math::Tolerance<float>());
}

TEST_F(GivensRotationTest, JacobiRotationAnnihilatesSymmetricOffDiagonal)
{
    float app = 4.0f, aqq = 1.0f, apq = 2.0f;
    auto g = math::ComputeJacobiRotation(app, aqq, apq);

    float offDiagonal = (g.c * g.c - g.s * g.s) * apq + g.c * g.s * (app - aqq);

    EXPECT_NEAR(offDiagonal, 0.0f, math::Tolerance<float>());
}

TEST_F(GivensRotationTest, JacobiRotationUnitarity)
{
    float app = 3.0f, aqq = 7.0f, apq = 2.0f;
    auto g = math::ComputeJacobiRotation(app, aqq, apq);

    EXPECT_NEAR(g.c * g.c + g.s * g.s, 1.0f, math::Tolerance<float>());
}

TEST_F(GivensRotationTest, JacobiRotationZeroOffDiagonalIsIdentity)
{
    auto g = math::ComputeJacobiRotation(3.0f, 5.0f, 0.0f);

    EXPECT_NEAR(g.c, 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(g.s, 0.0f, math::Tolerance<float>());
}

TEST_F(GivensRotationTest, NormPreservationOverSeededSweep)
{
    std::mt19937 prng{ 42u };
    std::uniform_real_distribution<float> dist{ -10.0f, 10.0f };

    std::array<float, 16> as{};
    std::array<float, 16> bs{};
    for (auto& v : as)
        v = dist(prng);
    for (auto& v : bs)
        v = dist(prng);

    for (std::size_t i = 0; i < as.size(); ++i)
    {
        float norm = std::sqrt(as[i] * as[i] + bs[i] * bs[i]);
        if (norm < 1e-30f)
            continue;

        auto g = math::ComputeGivens(as[i], bs[i]);
        float x = as[i];
        float y = bs[i];
        math::ApplyGivens(g, x, y);

        EXPECT_NEAR(std::sqrt(x * x + y * y), norm, math::Tolerance<float>());
        EXPECT_NEAR(y, 0.0f, math::Tolerance<float>());
    }
}
