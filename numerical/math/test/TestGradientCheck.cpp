#include "numerical/math/test_doubles/GradientCheck.hpp"
#include <cmath>
#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>
#include <limits>

namespace
{
    using Vector3 = math::Vector<float, 3>;

    float Cubic(const Vector3& x)
    {
        return x.at(0, 0) * x.at(0, 0) * x.at(0, 0) + 2.0f * x.at(1, 0) * x.at(1, 0) + 3.0f * x.at(2, 0);
    }

    Vector3 CubicGradient(const Vector3& x)
    {
        return Vector3{ { 3.0f * x.at(0, 0) * x.at(0, 0) }, { 4.0f * x.at(1, 0) }, { 3.0f } };
    }

    class TestGradientCheck
        : public ::testing::Test
    {
    public:
        const Vector3 x{ { 2.0f }, { -1.0f }, { 0.5f } };
    };
}

TEST_F(TestGradientCheck, central_difference_matches_analytic_gradient)
{
    const auto numeric = math::test::CentralDifferenceGradient(Cubic, x, 1e-2f);
    const auto analytic = CubicGradient(x);

    EXPECT_NEAR(numeric.at(0, 0), analytic.at(0, 0), 1e-3f);
    EXPECT_NEAR(numeric.at(1, 0), analytic.at(1, 0), 1e-3f);
    EXPECT_NEAR(numeric.at(2, 0), analytic.at(2, 0), 1e-3f);
}

TEST_F(TestGradientCheck, default_step_scales_with_component_magnitude)
{
    const float cbrtEps = std::cbrt(std::numeric_limits<float>::epsilon());

    const auto h = math::test::DefaultFiniteDifferenceSteps(Vector3{ { 0.1f }, { -100.0f }, { 1.0f } });

    EXPECT_FLOAT_EQ(h.at(0, 0), cbrtEps);
    EXPECT_FLOAT_EQ(h.at(1, 0), 100.0f * cbrtEps);
    EXPECT_FLOAT_EQ(h.at(2, 0), cbrtEps);
}

TEST_F(TestGradientCheck, expect_gradient_near_accepts_correct_gradient)
{
    math::test::ExpectGradientNear(CubicGradient(x), Cubic, x, 1e-3f);
}

TEST_F(TestGradientCheck, expect_gradient_near_reports_wrong_component)
{
    auto wrong = CubicGradient(x);
    wrong.at(1, 0) = 99.0f;

    EXPECT_NONFATAL_FAILURE(math::test::ExpectGradientNear(wrong, Cubic, x, 1e-2f, 1e-3f), "component[1]");
}
