#include "numerical/control_analysis/ContinuousToDiscrete.hpp"
#include "numerical/math/Tolerance.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    class TestContinuousToDiscrete : public ::testing::Test
    {
    protected:
        control_analysis::ContinuousToDiscrete<float, 2, 1, 1> c2d{};
        math::LinearTimeInvariant<float, 2, 1, 1> continuousSys{};
    };
}

TEST_F(TestContinuousToDiscrete, IntegratorZoh)
{
    continuousSys.A = math::SquareMatrix<float, 2>{};
    continuousSys.B = math::Matrix<float, 2, 1>{ 1.0f, 0.0f };
    continuousSys.C = math::Matrix<float, 1, 2>{ 1.0f, 0.0f };
    continuousSys.D = math::Matrix<float, 1, 1>{ 0.0f };

    const float ts{ 0.1f };
    auto result = c2d.Convert(continuousSys, ts, control_analysis::DiscretizationMethod::ZeroOrderHold);

    EXPECT_NEAR(result.A.at(0, 0), 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.B.at(0, 0), ts, math::Tolerance<float>());
}

TEST_F(TestContinuousToDiscrete, FirstOrderZohMatchesAnalytic)
{
    const float a{ 1.0f };
    const float b{ 1.0f };
    const float ts{ 0.5f };

    continuousSys.A = math::SquareMatrix<float, 2>{ -a, 0.0f, 0.0f, 0.0f };
    continuousSys.B = math::Matrix<float, 2, 1>{ b, 0.0f };
    continuousSys.C = math::Matrix<float, 1, 2>{ 1.0f, 0.0f };
    continuousSys.D = math::Matrix<float, 1, 1>{ 0.0f };

    auto result = c2d.Convert(continuousSys, ts, control_analysis::DiscretizationMethod::ZeroOrderHold);

    const float expectedAd{ std::exp(-a * ts) };
    const float expectedBd{ (b / a) * (1.0f - std::exp(-a * ts)) };

    EXPECT_NEAR(result.A.at(0, 0), expectedAd, math::Tolerance<float>());
    EXPECT_NEAR(result.B.at(0, 0), expectedBd, math::Tolerance<float>());
}

TEST_F(TestContinuousToDiscrete, ZohPreservesCAndD)
{
    continuousSys.A = math::SquareMatrix<float, 2>{ -1.0f, 0.0f, 0.0f, -2.0f };
    continuousSys.B = math::Matrix<float, 2, 1>{ 1.0f, 0.5f };
    continuousSys.C = math::Matrix<float, 1, 2>{ 3.0f, 4.0f };
    continuousSys.D = math::Matrix<float, 1, 1>{ 5.0f };

    auto result = c2d.Convert(continuousSys, 0.1f, control_analysis::DiscretizationMethod::ZeroOrderHold);

    EXPECT_NEAR(result.C.at(0, 0), 3.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.C.at(0, 1), 4.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.D.at(0, 0), 5.0f, math::Tolerance<float>());
}

TEST_F(TestContinuousToDiscrete, ZohPreservesStability)
{
    continuousSys.A = math::SquareMatrix<float, 2>{ -2.0f, 0.0f, 0.0f, -5.0f };
    continuousSys.B = math::Matrix<float, 2, 1>{ 1.0f, 1.0f };
    continuousSys.C = math::Matrix<float, 1, 2>{ 1.0f, 0.0f };
    continuousSys.D = math::Matrix<float, 1, 1>{ 0.0f };

    auto result = c2d.Convert(continuousSys, 0.1f, control_analysis::DiscretizationMethod::ZeroOrderHold);

    EXPECT_LT(std::abs(result.A.at(0, 0)), 1.0f);
    EXPECT_LT(std::abs(result.A.at(1, 1)), 1.0f);
}

TEST_F(TestContinuousToDiscrete, ForwardEulerFormula)
{
    continuousSys.A = math::SquareMatrix<float, 2>{ -1.0f, 0.0f, 0.0f, -2.0f };
    continuousSys.B = math::Matrix<float, 2, 1>{ 1.0f, 0.5f };
    continuousSys.C = math::Matrix<float, 1, 2>{ 1.0f, 0.0f };
    continuousSys.D = math::Matrix<float, 1, 1>{ 0.0f };

    const float ts{ 0.1f };
    auto result = c2d.Convert(continuousSys, ts, control_analysis::DiscretizationMethod::ForwardEuler);

    EXPECT_NEAR(result.A.at(0, 0), 1.0f + (-1.0f) * ts, math::Tolerance<float>());
    EXPECT_NEAR(result.A.at(1, 1), 1.0f + (-2.0f) * ts, math::Tolerance<float>());
    EXPECT_NEAR(result.B.at(0, 0), 1.0f * ts, math::Tolerance<float>());
    EXPECT_NEAR(result.B.at(1, 0), 0.5f * ts, math::Tolerance<float>());
    EXPECT_NEAR(result.C.at(0, 0), 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.D.at(0, 0), 0.0f, math::Tolerance<float>());
}

TEST_F(TestContinuousToDiscrete, BackwardEulerFormula)
{
    const float a{ 1.0f };
    const float b{ 1.0f };
    const float ts{ 0.1f };

    continuousSys.A = math::SquareMatrix<float, 2>{ -a, 0.0f, 0.0f, 0.0f };
    continuousSys.B = math::Matrix<float, 2, 1>{ b, 0.0f };
    continuousSys.C = math::Matrix<float, 1, 2>{ 1.0f, 0.0f };
    continuousSys.D = math::Matrix<float, 1, 1>{ 0.0f };

    auto result = c2d.Convert(continuousSys, ts, control_analysis::DiscretizationMethod::BackwardEuler);

    const float p{ 1.0f / (1.0f + a * ts) };
    const float expectedAd{ p };
    const float expectedBd{ p * b * ts };
    const float expectedCd{ 1.0f * p };
    const float expectedDd{ 0.0f + 1.0f * p * b * ts };

    EXPECT_NEAR(result.A.at(0, 0), expectedAd, math::Tolerance<float>());
    EXPECT_NEAR(result.B.at(0, 0), expectedBd, math::Tolerance<float>());
    EXPECT_NEAR(result.C.at(0, 0), expectedCd, math::Tolerance<float>());
    EXPECT_NEAR(result.D.at(0, 0), expectedDd, math::Tolerance<float>());
}

TEST_F(TestContinuousToDiscrete, BackwardEulerPreservesStability)
{
    continuousSys.A = math::SquareMatrix<float, 2>{ -0.5f, 0.0f, 0.0f, -0.5f };
    continuousSys.B = math::Matrix<float, 2, 1>{ 1.0f, 1.0f };
    continuousSys.C = math::Matrix<float, 1, 2>{ 1.0f, 0.0f };
    continuousSys.D = math::Matrix<float, 1, 1>{ 0.0f };

    auto result = c2d.Convert(continuousSys, 2.0f, control_analysis::DiscretizationMethod::BackwardEuler);

    EXPECT_LT(std::abs(result.A.at(0, 0)), 1.0f);
    EXPECT_LT(std::abs(result.A.at(1, 1)), 1.0f);
}

TEST_F(TestContinuousToDiscrete, TustinBilinearReference)
{
    const float a{ 2.0f };
    const float b{ 1.0f };
    const float ts{ 0.1f };
    const float alpha{ 2.0f / ts };

    continuousSys.A = math::SquareMatrix<float, 2>{ -a, 0.0f, 0.0f, 0.0f };
    continuousSys.B = math::Matrix<float, 2, 1>{ b, 0.0f };
    continuousSys.C = math::Matrix<float, 1, 2>{ 1.0f, 0.0f };
    continuousSys.D = math::Matrix<float, 1, 1>{ 0.0f };

    auto result = c2d.Convert(continuousSys, ts, control_analysis::DiscretizationMethod::Tustin);

    const float expectedAd{ (alpha - a) / (alpha + a) };
    const float expectedBd{ b * 2.0f / (alpha + a) };

    EXPECT_NEAR(result.A.at(0, 0), expectedAd, math::Tolerance<float>());
    EXPECT_NEAR(result.B.at(0, 0), expectedBd, math::Tolerance<float>());
}

TEST_F(TestContinuousToDiscrete, TustinPreservesStability)
{
    continuousSys.A = math::SquareMatrix<float, 2>{ -3.0f, 0.0f, 0.0f, -5.0f };
    continuousSys.B = math::Matrix<float, 2, 1>{ 1.0f, 1.0f };
    continuousSys.C = math::Matrix<float, 1, 2>{ 1.0f, 0.0f };
    continuousSys.D = math::Matrix<float, 1, 1>{ 0.0f };

    auto result = c2d.Convert(continuousSys, 0.05f, control_analysis::DiscretizationMethod::Tustin);

    EXPECT_LT(std::abs(result.A.at(0, 0)), 1.0f);
    EXPECT_LT(std::abs(result.A.at(1, 1)), 1.0f);
}

TEST_F(TestContinuousToDiscrete, TustinDcGainPreserved)
{
    const float a{ 3.0f };
    const float b{ 1.0f };
    const float ts{ 0.05f };

    continuousSys.A = math::SquareMatrix<float, 2>{ -a, 0.0f, 0.0f, 0.0f };
    continuousSys.B = math::Matrix<float, 2, 1>{ b, 0.0f };
    continuousSys.C = math::Matrix<float, 1, 2>{ 1.0f, 0.0f };
    continuousSys.D = math::Matrix<float, 1, 1>{ 0.0f };

    const float continuousDcGain{ b / a };

    auto discrete = c2d.Convert(continuousSys, ts, control_analysis::DiscretizationMethod::Tustin);

    const float oneMinusAd{ 1.0f - discrete.A.at(0, 0) };
    const float discreteDcGain{ discrete.C.at(0, 0) * discrete.B.at(0, 0) / oneMinusAd + discrete.D.at(0, 0) };

    EXPECT_NEAR(discreteDcGain, continuousDcGain, math::Tolerance<float>());
}

TEST_F(TestContinuousToDiscrete, SmallTsMethodsConverge)
{
    const float a{ 1.0f };
    const float ts{ 1e-4f };

    continuousSys.A = math::SquareMatrix<float, 2>{ -a, 0.0f, 0.0f, 0.0f };
    continuousSys.B = math::Matrix<float, 2, 1>{ 1.0f, 0.0f };
    continuousSys.C = math::Matrix<float, 1, 2>{ 1.0f, 0.0f };
    continuousSys.D = math::Matrix<float, 1, 1>{ 0.0f };

    auto zoh = c2d.Convert(continuousSys, ts, control_analysis::DiscretizationMethod::ZeroOrderHold);
    auto tustin = c2d.Convert(continuousSys, ts, control_analysis::DiscretizationMethod::Tustin);
    auto euler = c2d.Convert(continuousSys, ts, control_analysis::DiscretizationMethod::ForwardEuler);

    EXPECT_NEAR(zoh.A.at(0, 0), tustin.A.at(0, 0), 1e-3f);
    EXPECT_NEAR(zoh.A.at(0, 0), euler.A.at(0, 0), 1e-3f);
    EXPECT_NEAR(zoh.B.at(0, 0), tustin.B.at(0, 0), 1e-3f);
    EXPECT_NEAR(zoh.B.at(0, 0), euler.B.at(0, 0), 1e-3f);
}

TEST_F(TestContinuousToDiscrete, DcGainPreserved)
{
    const float a{ 2.0f };
    const float b{ 1.0f };
    const float ts{ 0.05f };

    continuousSys.A = math::SquareMatrix<float, 2>{ -a, 0.0f, 0.0f, 0.0f };
    continuousSys.B = math::Matrix<float, 2, 1>{ b, 0.0f };
    continuousSys.C = math::Matrix<float, 1, 2>{ 1.0f, 0.0f };
    continuousSys.D = math::Matrix<float, 1, 1>{ 0.0f };

    const float continuousDcGain{ b / a };

    auto discrete = c2d.Convert(continuousSys, ts, control_analysis::DiscretizationMethod::ZeroOrderHold);

    const float oneMinusAd{ 1.0f - discrete.A.at(0, 0) };
    const float discreteDcGain{ discrete.C.at(0, 0) * discrete.B.at(0, 0) / oneMinusAd + discrete.D.at(0, 0) };

    EXPECT_NEAR(discreteDcGain, continuousDcGain, math::Tolerance<float>());
}

TEST_F(TestContinuousToDiscrete, DeterminismSameInputSameOutput)
{
    continuousSys.A = math::SquareMatrix<float, 2>{ -1.0f, 0.5f, 0.0f, -2.0f };
    continuousSys.B = math::Matrix<float, 2, 1>{ 1.0f, 1.0f };
    continuousSys.C = math::Matrix<float, 1, 2>{ 1.0f, 0.0f };
    continuousSys.D = math::Matrix<float, 1, 1>{ 0.0f };

    const float ts{ 0.1f };
    auto result1 = c2d.Convert(continuousSys, ts, control_analysis::DiscretizationMethod::ZeroOrderHold);
    auto result2 = c2d.Convert(continuousSys, ts, control_analysis::DiscretizationMethod::ZeroOrderHold);

    EXPECT_FLOAT_EQ(result1.A.at(0, 0), result2.A.at(0, 0));
    EXPECT_FLOAT_EQ(result1.A.at(0, 1), result2.A.at(0, 1));
    EXPECT_FLOAT_EQ(result1.B.at(0, 0), result2.B.at(0, 0));
    EXPECT_FLOAT_EQ(result1.B.at(1, 0), result2.B.at(1, 0));
}
