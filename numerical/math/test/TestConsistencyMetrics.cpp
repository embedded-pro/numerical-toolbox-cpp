#include "numerical/math/ConsistencyMetrics.hpp"
#include "numerical/math/Tolerance.hpp"
#include <gtest/gtest.h>

namespace
{
    class ConsistencyMetrics1DTest
        : public ::testing::Test
    {
    protected:
        using Metrics = math::ConsistencyMetrics<float, 1>;
        using Vec1 = math::Vector<float, 1>;
        using Mat1 = math::Matrix<float, 1, 1>;
    };

    class ConsistencyMetrics2DTest
        : public ::testing::Test
    {
    protected:
        using Metrics = math::ConsistencyMetrics<float, 2>;
        using Vec2 = math::Vector<float, 2>;
        using Mat2 = math::Matrix<float, 2, 2>;
    };

    class ConsistencyMetrics3DTest
        : public ::testing::Test
    {
    protected:
        using Metrics = math::ConsistencyMetrics<float, 3>;
        using Vec3 = math::Vector<float, 3>;
        using Mat3 = math::Matrix<float, 3, 3>;
    };
}

TEST_F(ConsistencyMetrics1DTest, NeesScalarIdentityCovariance)
{
    Vec1 error;
    error.at(0, 0) = 2.0f;
    Mat1 cov;
    cov.at(0, 0) = 1.0f;

    auto result = Metrics::Nees(error, cov);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*result, 4.0f, math::Tolerance<float>());
}

TEST_F(ConsistencyMetrics1DTest, NisScalarIdentityCovariance)
{
    Vec1 innovation;
    innovation.at(0, 0) = 3.0f;
    Mat1 cov;
    cov.at(0, 0) = 1.0f;

    auto result = Metrics::Nis(innovation, cov);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*result, 9.0f, math::Tolerance<float>());
}

TEST_F(ConsistencyMetrics1DTest, NeesScaledCovariance)
{
    Vec1 error;
    error.at(0, 0) = 2.0f;
    Mat1 cov;
    cov.at(0, 0) = 4.0f;

    auto result = Metrics::Nees(error, cov);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*result, 1.0f, math::Tolerance<float>());
}

TEST_F(ConsistencyMetrics1DTest, NeesSingularCovarianceReturnsNullopt)
{
    Vec1 error;
    error.at(0, 0) = 1.0f;
    Mat1 cov;
    cov.at(0, 0) = 0.0f;

    auto result = Metrics::Nees(error, cov);

    EXPECT_FALSE(result.has_value());
}

TEST_F(ConsistencyMetrics1DTest, NisSingularCovarianceReturnsNullopt)
{
    Vec1 innovation;
    innovation.at(0, 0) = 1.0f;
    Mat1 cov;
    cov.at(0, 0) = 0.0f;

    auto result = Metrics::Nis(innovation, cov);

    EXPECT_FALSE(result.has_value());
}

TEST_F(ConsistencyMetrics1DTest, IsConsistentInsideBounds)
{
    const float midpoint = (math::detail::kChi2Lo95[0] + math::detail::kChi2Hi95[0]) / 2.0f;

    EXPECT_TRUE(Metrics::IsConsistent(midpoint));
}

TEST_F(ConsistencyMetrics1DTest, IsConsistentAtExactLowerBound)
{
    EXPECT_TRUE(Metrics::IsConsistent(math::detail::kChi2Lo95[0]));
}

TEST_F(ConsistencyMetrics1DTest, IsConsistentAtExactUpperBound)
{
    EXPECT_TRUE(Metrics::IsConsistent(math::detail::kChi2Hi95[0]));
}

TEST_F(ConsistencyMetrics1DTest, IsConsistentOutsideBoundsHigh)
{
    const float aboveHi = math::detail::kChi2Hi95[0] + 1.0f;

    EXPECT_FALSE(Metrics::IsConsistent(aboveHi));
}

TEST_F(ConsistencyMetrics1DTest, IsConsistentOutsideBoundsLow)
{
    EXPECT_FALSE(Metrics::IsConsistent(-1.0f));
}

TEST_F(ConsistencyMetrics1DTest, IsTimeAveragedConsistentZeroSamplesReturnsFalse)
{
    EXPECT_FALSE(Metrics::IsTimeAveragedConsistent(1.0f, 0));
}

TEST_F(ConsistencyMetrics1DTest, IsTimeAveragedConsistentMultipleSamplesInsideBounds)
{
    const float lo = math::detail::kChi2Lo95[1] / 2.0f;
    const float hi = math::detail::kChi2Hi95[1] / 2.0f;
    const float midpoint = (lo + hi) / 2.0f;

    EXPECT_TRUE(Metrics::IsTimeAveragedConsistent(midpoint, 2));
}

TEST_F(ConsistencyMetrics1DTest, IsTimeAveragedConsistentInconsistentOutOfBounds)
{
    EXPECT_FALSE(Metrics::IsTimeAveragedConsistent(0.0f, 2));
}

TEST_F(ConsistencyMetrics2DTest, Nees2DIdentityCovariance)
{
    Vec2 error;
    error.at(0, 0) = 1.0f;
    error.at(1, 0) = 2.0f;
    Mat2 cov;
    cov.at(0, 0) = 1.0f;
    cov.at(0, 1) = 0.0f;
    cov.at(1, 0) = 0.0f;
    cov.at(1, 1) = 1.0f;

    auto result = Metrics::Nees(error, cov);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*result, 5.0f, math::Tolerance<float>());
}

TEST_F(ConsistencyMetrics2DTest, Nis2DScaledDiagonalCovariance)
{
    Vec2 innovation;
    innovation.at(0, 0) = 2.0f;
    innovation.at(1, 0) = 4.0f;
    Mat2 cov;
    cov.at(0, 0) = 4.0f;
    cov.at(0, 1) = 0.0f;
    cov.at(1, 0) = 0.0f;
    cov.at(1, 1) = 16.0f;

    auto result = Metrics::Nis(innovation, cov);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*result, 2.0f, math::Tolerance<float>());
}

TEST_F(ConsistencyMetrics2DTest, Nees2DFullCovarianceHandComputed)
{
    Vec2 error;
    error.at(0, 0) = 1.0f;
    error.at(1, 0) = 0.0f;
    Mat2 cov;
    cov.at(0, 0) = 2.0f;
    cov.at(0, 1) = 1.0f;
    cov.at(1, 0) = 1.0f;
    cov.at(1, 1) = 2.0f;

    auto result = Metrics::Nees(error, cov);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*result, 2.0f / 3.0f, math::Tolerance<float>());
}

TEST_F(ConsistencyMetrics2DTest, Nis2DSingularCovarianceReturnsNullopt)
{
    Vec2 innovation;
    innovation.at(0, 0) = 1.0f;
    innovation.at(1, 0) = 1.0f;
    Mat2 cov;
    cov.at(0, 0) = 0.0f;
    cov.at(0, 1) = 0.0f;
    cov.at(1, 0) = 0.0f;
    cov.at(1, 1) = 1.0f;

    auto result = Metrics::Nis(innovation, cov);

    EXPECT_FALSE(result.has_value());
}

TEST_F(ConsistencyMetrics2DTest, IsConsistent2DWithinBounds)
{
    const float midpoint = (math::detail::kChi2Lo95[1] + math::detail::kChi2Hi95[1]) / 2.0f;

    EXPECT_TRUE(Metrics::IsConsistent(midpoint));
}

TEST_F(ConsistencyMetrics3DTest, Nees3DIdentityCovariance)
{
    Vec3 error;
    error.at(0, 0) = 1.0f;
    error.at(1, 0) = 1.0f;
    error.at(2, 0) = 1.0f;
    Mat3 cov;
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            cov.at(i, j) = (i == j) ? 1.0f : 0.0f;

    auto result = Metrics::Nees(error, cov);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*result, 3.0f, math::Tolerance<float>());
}

TEST_F(ConsistencyMetrics3DTest, Nis3DScaledDiagonalCovariance)
{
    Vec3 innovation;
    innovation.at(0, 0) = 1.0f;
    innovation.at(1, 0) = 0.0f;
    innovation.at(2, 0) = 0.0f;
    Mat3 cov;
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            cov.at(i, j) = (i == j) ? 2.0f : 0.0f;

    auto result = Metrics::Nis(innovation, cov);

    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(*result, 0.5f, math::Tolerance<float>());
}

TEST_F(ConsistencyMetrics3DTest, IsTimeAveragedConsistentWithOneSample)
{
    const float midpoint = (math::detail::kChi2Lo95[2] + math::detail::kChi2Hi95[2]) / 2.0f;

    EXPECT_TRUE(Metrics::IsTimeAveragedConsistent(midpoint, 1));
}

TEST_F(ConsistencyMetrics3DTest, IsTimeAveragedConsistentLargeDofWilsonHilferty)
{
    // dof=15 > kMaxChiSquareDim; WH bounds ≈ [1.248, 5.499] after dividing by 5
    EXPECT_TRUE(Metrics::IsTimeAveragedConsistent(2.373f, 5));
    EXPECT_FALSE(Metrics::IsTimeAveragedConsistent(6.0f, 5));
}

TEST_F(ConsistencyMetrics3DTest, IsTimeAveragedConsistentInconsistentOutOfBounds)
{
    const float aboveHi = math::detail::kChi2Hi95[math::detail::kMaxChiSquareDim - 1] / 1.0f + 1.0f;

    EXPECT_FALSE(Metrics::IsTimeAveragedConsistent(aboveHi, 1));
}

TEST_F(ConsistencyMetrics1DTest, IsTimeAveragedConsistentLargeNumSamplesUsesApproximation)
{
    // dof=100 >> kMaxChiSquareDim; WH bounds ≈ [0.742, 1.296] — old code gave hi≈0.205 (wrong)
    EXPECT_TRUE(Metrics::IsTimeAveragedConsistent(1.0f, 100));
    EXPECT_FALSE(Metrics::IsTimeAveragedConsistent(1.4f, 100));
}
