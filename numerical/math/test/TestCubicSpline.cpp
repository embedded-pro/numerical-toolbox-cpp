#include "numerical/math/CubicSpline.hpp"
#include <array>
#include <cstddef>
#include <gtest/gtest.h>

namespace
{
    using Spline = math::CubicSpline<float, 3, 8>;
    using Value = Spline::Value;
    using Boundary = math::SplineBoundary<float, 3>;

    class TestCubicSpline : public ::testing::Test
    {
    protected:
        static constexpr float tolerance{ 1e-4f };
        Spline::Knots knots{ 0.0f, 1.0f, 2.5f, 3.0f, 4.5f };
        Spline::Values values{ Value{ 0.0f, 0.5f, 0.0f }, Value{ 1.0f, 0.2f, 0.0f }, Value{ -0.5f, 0.2f, 0.0f },
            Value{ 0.3f, -0.4f, 0.0f }, Value{ 2.0f, 0.0f, 0.0f } };
        Spline spline;

        void SetUp() override
        {
            ASSERT_TRUE(spline.Fit(knots, values, 5));
        }

        static void ExpectVectorNear(const Value& actual, const Value& expected, float eps)
        {
            for (std::size_t j = 0; j < 3; ++j)
                EXPECT_NEAR(actual[j], expected[j], eps) << "channel " << j;
        }
    };
}

TEST_F(TestCubicSpline, passes_through_every_knot)
{
    for (std::size_t k = 0; k < 5; ++k)
        ExpectVectorNear(spline.Evaluate(knots[k]).value, values[k], 1e-5f);
}

TEST_F(TestCubicSpline, first_and_second_derivatives_continuous_at_interior_knots)
{
    constexpr float delta{ 1e-4f };

    for (std::size_t k = 1; k < 4; ++k)
    {
        const auto before{ spline.Evaluate(knots[k] - delta) };
        const auto after{ spline.Evaluate(knots[k] + delta) };

        ExpectVectorNear(before.firstDerivative, after.firstDerivative, 1e-2f);
        ExpectVectorNear(before.secondDerivative, after.secondDerivative, 1e-2f);
    }
}

TEST_F(TestCubicSpline, knot_second_derivatives_match_reference)
{
    const std::array<float, 5> channel0{ 5.665169f, -5.330337f, 5.991011f, -0.737079f, -1.898127f };
    const std::array<float, 5> channel1{ -1.666292f, 1.532584f, -2.797753f, 3.384270f, -2.225468f };

    for (std::size_t k = 0; k < 5; ++k)
    {
        const auto secondDerivative{ spline.Evaluate(knots[k]).secondDerivative };

        EXPECT_NEAR(secondDerivative[0], channel0[k], tolerance);
        EXPECT_NEAR(secondDerivative[1], channel1[k], tolerance);
        EXPECT_EQ(secondDerivative[2], 0.0f);
    }
}

TEST_F(TestCubicSpline, clamped_end_slopes_are_honoured)
{
    ASSERT_TRUE(spline.Fit(knots, values, 5, Boundary{ math::SplineEnd::Clamped, Value{ 0.4f, 0.0f, 0.0f }, Value{ -0.7f, 0.0f, 0.0f } }));

    EXPECT_NEAR(spline.Evaluate(0.0f).firstDerivative[0], 0.4f, tolerance);
    EXPECT_NEAR(spline.Evaluate(4.5f).firstDerivative[0], -0.7f, tolerance);
    EXPECT_NEAR(spline.Evaluate(0.0f).secondDerivative[0], 4.296629f, tolerance);
    EXPECT_NEAR(spline.Evaluate(4.5f).secondDerivative[0], -3.637453f, tolerance);
}

TEST_F(TestCubicSpline, natural_ends_have_zero_second_derivative)
{
    ASSERT_TRUE(spline.Fit(knots, values, 5, Boundary{ math::SplineEnd::Natural }));

    ExpectVectorNear(spline.Evaluate(0.0f).secondDerivative, Value{}, tolerance);
    ExpectVectorNear(spline.Evaluate(4.5f).secondDerivative, Value{}, tolerance);
    EXPECT_NEAR(spline.Evaluate(0.0f).firstDerivative[0], 1.680287f, tolerance);
    EXPECT_NEAR(spline.Evaluate(4.5f).firstDerivative[0], 0.783154f, tolerance);
}

TEST_F(TestCubicSpline, collinear_knots_reproduce_straight_line)
{
    const Spline::Knots line{ 0.0f, 0.5f, 1.0f, 1.5f, 2.0f };
    Spline::Values onLine{};
    for (std::size_t k = 0; k < 5; ++k)
        onLine[k] = Value{ 1.0f + 0.8f * line[k], 1.0f + 0.8f * line[k], 1.0f + 0.8f * line[k] };
    const Value slope{ 0.8f, 0.8f, 0.8f };

    ASSERT_TRUE(spline.Fit(line, onLine, 5, Boundary{ math::SplineEnd::Clamped, slope, slope }));

    const auto point{ spline.Evaluate(0.7f) };
    ExpectVectorNear(point.value, Value{ 1.56f, 1.56f, 1.56f }, 1e-5f);
    ExpectVectorNear(point.firstDerivative, slope, 1e-5f);
    for (float x : { 0.0f, 0.3f, 0.7f, 1.2f, 2.0f })
        ExpectVectorNear(spline.Evaluate(x).secondDerivative, Value{}, 1e-5f);
}

TEST_F(TestCubicSpline, three_knot_reference_values)
{
    ASSERT_TRUE(spline.Fit(Spline::Knots{ 0.0f, 1.0f, 2.0f }, Spline::Values{ Value{}, Value{ 1.0f, 0.0f, 0.0f }, Value{} }, 3));

    const auto half{ spline.Evaluate(0.5f) };
    EXPECT_NEAR(half.value[0], 0.5f, 1e-5f);
    EXPECT_NEAR(half.firstDerivative[0], 1.5f, 1e-5f);
    EXPECT_NEAR(half.secondDerivative[0], 0.0f, 1e-5f);
    EXPECT_NEAR(spline.Evaluate(0.0f).secondDerivative[0], 6.0f, 1e-5f);
    EXPECT_NEAR(spline.Evaluate(1.0f).secondDerivative[0], -6.0f, 1e-5f);
    EXPECT_NEAR(spline.Evaluate(2.0f).secondDerivative[0], 6.0f, 1e-5f);
    EXPECT_NEAR(spline.Evaluate(1.0f).firstDerivative[0], 0.0f, 1e-5f);
}

TEST_F(TestCubicSpline, evaluate_clamps_outside_domain)
{
    const auto below{ spline.Evaluate(-1.0f) };
    const auto start{ spline.Evaluate(0.0f) };
    const auto above{ spline.Evaluate(5.5f) };
    const auto end{ spline.Evaluate(4.5f) };

    EXPECT_EQ(spline.First(), 0.0f);
    EXPECT_EQ(spline.Last(), 4.5f);
    for (std::size_t j = 0; j < 3; ++j)
    {
        EXPECT_EQ(below.value[j], start.value[j]);
        EXPECT_EQ(below.firstDerivative[j], start.firstDerivative[j]);
        EXPECT_EQ(above.value[j], end.value[j]);
        EXPECT_EQ(above.secondDerivative[j], end.secondDerivative[j]);
    }
}

TEST_F(TestCubicSpline, fit_rejects_invalid_knots)
{
    EXPECT_FALSE(spline.Fit(knots, values, 1));
    EXPECT_FALSE(spline.Fit(knots, values, 9));
    EXPECT_FALSE(spline.Fit(Spline::Knots{ 0.0f, 1.0f, 1.0f, 2.0f }, values, 4));
}
