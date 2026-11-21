#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Tolerance.hpp"
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    class Geometry3DTest : public ::testing::Test
    {
    protected:
        static void ExpectNear(const math::Matrix3<float>& actual, const math::Matrix3<float>& expected)
        {
            for (std::size_t i = 0; i < 3; ++i)
                for (std::size_t j = 0; j < 3; ++j)
                    EXPECT_NEAR(actual.at(i, j), expected.at(i, j), math::Tolerance<float>());
        }

        float angle{ 0.7f };
    };
}

TEST_F(Geometry3DTest, ElementaryRotationsMatchTheRotationAboutTheirAxis)
{
    ExpectNear(math::RotationX(angle), math::RotationAboutAxis(math::Vector3<float>{ 1.0f, 0.0f, 0.0f }, angle));
    ExpectNear(math::RotationY(angle), math::RotationAboutAxis(math::Vector3<float>{ 0.0f, 1.0f, 0.0f }, angle));
    ExpectNear(math::RotationZ(angle), math::RotationAboutAxis(math::Vector3<float>{ 0.0f, 0.0f, 1.0f }, angle));
}

TEST_F(Geometry3DTest, RotationZTurnsTheXAxisTowardsTheYAxis)
{
    const auto turned = math::RotationZ(std::numbers::pi_v<float> / 2.0f) * math::Vector3<float>{ 1.0f, 0.0f, 0.0f };

    EXPECT_NEAR(turned.at(0, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(turned.at(1, 0), 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(turned.at(2, 0), 0.0f, math::Tolerance<float>());
}
