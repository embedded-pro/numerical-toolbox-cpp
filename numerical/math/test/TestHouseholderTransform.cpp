#include "numerical/math/HouseholderTransform.hpp"
#include "numerical/math/Tolerance.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    class HouseholderTransformTest : public ::testing::Test
    {
    protected:
        math::Vector<float, 4> Reflect(const math::Vector<float, 4>& x, const math::Vector<float, 4>& v, float beta)
        {
            float dot = 0.0f;
            for (std::size_t i = 0; i < 4; ++i)
                dot += v.at(i, 0) * x.at(i, 0);

            math::Vector<float, 4> result;
            for (std::size_t i = 0; i < 4; ++i)
                result.at(i, 0) = x.at(i, 0) - beta * dot * v.at(i, 0);
            return result;
        }
    };
}


TEST_F(HouseholderTransformTest, ZerosEntriesBelowPivot)
{
    math::Vector<float, 4> x{ { 4.0f }, { 3.0f }, { 0.0f }, { 0.0f } };
    math::Vector<float, 4> v;
    float beta{};

    math::HouseholderVector(x, 0, v, beta);
    auto reflected = Reflect(x, v, beta);

    EXPECT_NEAR(reflected.at(1, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(reflected.at(2, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(reflected.at(3, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(std::abs(reflected.at(0, 0)), 5.0f, math::Tolerance<float>());
}

TEST_F(HouseholderTransformTest, RespectsStartOffset)
{
    math::Vector<float, 4> x{ { 9.0f }, { 0.0f }, { 3.0f }, { 4.0f } };
    math::Vector<float, 4> v;
    float beta{};

    math::HouseholderVector(x, 1, v, beta);
    auto reflected = Reflect(x, v, beta);

    EXPECT_NEAR(reflected.at(0, 0), 9.0f, math::Tolerance<float>());
    EXPECT_NEAR(std::abs(reflected.at(1, 0)), 5.0f, math::Tolerance<float>());
    EXPECT_NEAR(reflected.at(2, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(reflected.at(3, 0), 0.0f, math::Tolerance<float>());
}

TEST_F(HouseholderTransformTest, ZeroSubvectorYieldsZeroBeta)
{
    math::Vector<float, 4> x{ { 2.0f }, { 0.0f }, { 0.0f }, { 0.0f } };
    math::Vector<float, 4> v;
    float beta{ 1.0f };

    math::HouseholderVector(x, 0, v, beta);

    EXPECT_NEAR(beta, 0.0f, math::Tolerance<float>());
}

TEST_F(HouseholderTransformTest, ApplyReflectorLeftZerosSubcolumn)
{
    math::Matrix<float, 3, 3> a{ 12.0f, -51.0f, 4.0f,
                                  6.0f, 167.0f, -68.0f,
                                 -4.0f,  24.0f, -41.0f };

    math::Vector<float, 3> col0{ { 12.0f }, { 6.0f }, { -4.0f } };
    math::Vector<float, 3> v;
    float beta{};
    math::HouseholderVector(col0, 0, v, beta);

    math::ApplyReflectorLeft(a, v, beta, 0, 0);

    EXPECT_NEAR(a.at(0, 0), 14.0f, math::Tolerance<float>());
    EXPECT_NEAR(a.at(1, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(a.at(2, 0), 0.0f, math::Tolerance<float>());
}

TEST_F(HouseholderTransformTest, ApplyReflectorLeftRespectsColStart)
{
    math::Matrix<float, 3, 3> a{ 12.0f, -51.0f, 4.0f,
                                  6.0f, 167.0f, -68.0f,
                                 -4.0f,  24.0f, -41.0f };

    math::Vector<float, 3> col0{ { 12.0f }, { 6.0f }, { -4.0f } };
    math::Vector<float, 3> v;
    float beta{};
    math::HouseholderVector(col0, 0, v, beta);

    math::ApplyReflectorLeft(a, v, beta, 0, 1);

    EXPECT_NEAR(a.at(0, 0), 12.0f, math::Tolerance<float>());
    EXPECT_NEAR(a.at(1, 0), 6.0f, math::Tolerance<float>());
    EXPECT_NEAR(a.at(2, 0), -4.0f, math::Tolerance<float>());
}

TEST_F(HouseholderTransformTest, ApplyReflectorLeftIsInvolution)
{
    math::Matrix<float, 3, 3> a{ 12.0f, -51.0f, 4.0f,
                                  6.0f, 167.0f, -68.0f,
                                 -4.0f,  24.0f, -41.0f };

    math::Vector<float, 3> col0{ { 12.0f }, { 6.0f }, { -4.0f } };
    math::Vector<float, 3> v;
    float beta{};
    math::HouseholderVector(col0, 0, v, beta);

    math::ApplyReflectorLeft(a, v, beta, 0, 0);
    math::ApplyReflectorLeft(a, v, beta, 0, 0);

    EXPECT_NEAR(a.at(0, 0), 12.0f, math::Tolerance<float>());
    EXPECT_NEAR(a.at(1, 0), 6.0f, math::Tolerance<float>());
    EXPECT_NEAR(a.at(2, 0), -4.0f, math::Tolerance<float>());
}

TEST_F(HouseholderTransformTest, ApplyReflectorRightZerosSubrow)
{
    math::Matrix<float, 2, 3> b{ 3.0f,  0.0f,  8.0f,
                                  5.0f, 12.0f, 16.0f };

    math::Vector<float, 3> row0{ { 0.0f }, { 0.0f }, { 8.0f } };
    math::Vector<float, 3> v;
    float beta{};
    math::HouseholderVector(row0, 1, v, beta);

    math::ApplyReflectorRight(b, v, beta, 0, 1);

    EXPECT_NEAR(b.at(0, 0), 3.0f, math::Tolerance<float>());
    EXPECT_NEAR(b.at(0, 1), 8.0f, math::Tolerance<float>());
    EXPECT_NEAR(b.at(0, 2), 0.0f, math::Tolerance<float>());
}

TEST_F(HouseholderTransformTest, ApplyReflectorRightTransformsAllRows)
{
    math::Matrix<float, 2, 3> b{ 3.0f,  0.0f,  8.0f,
                                  5.0f, 12.0f, 16.0f };

    math::Vector<float, 3> row0{ { 0.0f }, { 0.0f }, { 8.0f } };
    math::Vector<float, 3> v;
    float beta{};
    math::HouseholderVector(row0, 1, v, beta);

    math::ApplyReflectorRight(b, v, beta, 0, 1);

    EXPECT_NEAR(b.at(1, 1), 16.0f, math::Tolerance<float>());
    EXPECT_NEAR(b.at(1, 2), 12.0f, math::Tolerance<float>());
}

TEST_F(HouseholderTransformTest, ApplyReflectorRightIsInvolution)
{
    math::Matrix<float, 2, 3> b{ 3.0f,  0.0f,  8.0f,
                                  5.0f, 12.0f, 16.0f };

    math::Vector<float, 3> row0{ { 0.0f }, { 0.0f }, { 8.0f } };
    math::Vector<float, 3> v;
    float beta{};
    math::HouseholderVector(row0, 1, v, beta);

    math::ApplyReflectorRight(b, v, beta, 0, 1);
    math::ApplyReflectorRight(b, v, beta, 0, 1);

    EXPECT_NEAR(b.at(0, 1), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(b.at(0, 2), 8.0f, math::Tolerance<float>());
}
