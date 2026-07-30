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
