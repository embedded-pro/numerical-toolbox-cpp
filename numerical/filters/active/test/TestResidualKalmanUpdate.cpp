#include "numerical/filters/active/ResidualKalmanUpdate.hpp"
#include "numerical/math/AngleUtilities.hpp"
#include <cstddef>
#include <gtest/gtest.h>

namespace
{
    using Scalar = filters::ResidualKalmanUpdate<float, 2, 1>;
    using Vector = filters::ResidualKalmanUpdate<float, 2, 2>;
    using Angle = filters::ResidualKalmanUpdate<float, 1, 1>;
    using filters::KalmanUpdateStatus;

    class TestResidualKalmanUpdate
        : public ::testing::Test
    {
    protected:
        static void ExpectState(const Scalar::StateVector& actual, float first, float second)
        {
            EXPECT_NEAR(actual.at(0, 0), first, 1e-5f);
            EXPECT_NEAR(actual.at(1, 0), second, 1e-5f);
        }

        static void ExpectCovariance(const Scalar::Covariance& actual, float diagonal0, float offDiagonal, float diagonal1)
        {
            EXPECT_NEAR(actual.at(0, 0), diagonal0, 1e-5f);
            EXPECT_NEAR(actual.at(0, 1), offDiagonal, 1e-5f);
            EXPECT_NEAR(actual.at(1, 0), offDiagonal, 1e-5f);
            EXPECT_NEAR(actual.at(1, 1), diagonal1, 1e-5f);
        }

        Scalar::StateVector x{ 1.0f, 0.5f };
        Scalar::Covariance P{ { 0.5f, 0.1f }, { 0.1f, 0.3f } };
        Scalar::Jacobian H{ { 1.0f, 0.0f } };
        Scalar::MeasurementNoise R{ { 0.2f } };
        Scalar::Residual r{ 0.4f };
    };
}

TEST_F(TestResidualKalmanUpdate, scalar_update_matches_the_kalman_equations)
{
    const auto result{ Scalar::Apply(x, P, r, H, R) };

    EXPECT_EQ(result.status, KalmanUpdateStatus::Applied);
    EXPECT_NEAR(result.nis, 0.228571f, 1e-5f);
    ExpectState(x, 1.285714f, 0.557143f);
    ExpectCovariance(P, 0.142857f, 0.028571f, 0.285714f);
}

TEST_F(TestResidualKalmanUpdate, vector_update_matches_the_kalman_equations)
{
    const Vector::Jacobian jacobian{ { 1.0f, 0.0f }, { 1.0f, 1.0f } };
    const Vector::MeasurementNoise noise{ { 0.2f, 0.0f }, { 0.0f, 0.1f } };

    const auto result{ Vector::Apply(x, P, Vector::Residual{ 0.4f, -0.3f }, jacobian, noise) };

    EXPECT_EQ(result.status, KalmanUpdateStatus::Applied);
    EXPECT_NEAR(result.nis, 0.934146f, 1e-5f);
    ExpectState(x, 1.097561f, 0.212195f);
    ExpectCovariance(P, 0.092683f, -0.063415f, 0.117073f);
}

TEST_F(TestResidualKalmanUpdate, nis_matches_the_applied_update)
{
    const auto nis{ Scalar::Nis(r, H, P, R) };

    ASSERT_TRUE(nis.has_value());
    EXPECT_NEAR(*nis, 0.228571f, 1e-5f);
    EXPECT_EQ(*nis, Scalar::Apply(x, P, r, H, R).nis);
}

TEST_F(TestResidualKalmanUpdate, gate_rejects_an_outlier_without_touching_the_state)
{
    const auto result{ Scalar::Apply(x, P, Scalar::Residual{ 3.0f }, H, R, 6.63f) };

    EXPECT_EQ(result.status, KalmanUpdateStatus::Gated);
    EXPECT_NEAR(result.nis, 12.857143f, 1e-4f);
    ExpectState(x, 1.0f, 0.5f);
    ExpectCovariance(P, 0.5f, 0.1f, 0.3f);
}

TEST_F(TestResidualKalmanUpdate, singular_innovation_covariance_is_reported)
{
    Scalar::Covariance zero{};
    const Scalar::MeasurementNoise noNoise{};

    const auto result{ Scalar::Apply(x, zero, r, H, noNoise) };

    EXPECT_EQ(result.status, KalmanUpdateStatus::Singular);
    EXPECT_EQ(result.nis, 0.0f);
    ExpectState(x, 1.0f, 0.5f);
    ExpectCovariance(zero, 0.0f, 0.0f, 0.0f);
    EXPECT_FALSE(Scalar::Nis(r, H, zero, noNoise).has_value());
}

TEST_F(TestResidualKalmanUpdate, wrapped_residual_keeps_an_angle_estimate_sane)
{
    Angle::StateVector heading{ 3.1f };
    Angle::Covariance variance{ { 0.04f } };
    const float measured{ -3.1f };

    const auto result{ Angle::Apply(heading, variance, Angle::Residual{ math::AngleDifference(measured, heading.at(0, 0)) }, Angle::Jacobian{ { 1.0f } }, Angle::MeasurementNoise{ { 0.01f } }) };
    heading.at(0, 0) = math::WrapToPi(heading.at(0, 0));

    EXPECT_NEAR(heading.at(0, 0), -3.116637f, 1e-5f);
    EXPECT_NEAR(variance.at(0, 0), 0.008f, 1e-6f);
    EXPECT_NEAR(result.nis, 0.138396f, 1e-4f);
}

TEST_F(TestResidualKalmanUpdate, joseph_form_stays_symmetric)
{
    for (std::size_t k = 0; k < 100; ++k)
        Scalar::Apply(x, P, r, H, R);

    EXPECT_EQ(P.at(0, 1), P.at(1, 0));
    EXPECT_GT(P.at(0, 0) + P.at(1, 1), 0.0f);
    EXPECT_GT(P.at(0, 0) * P.at(1, 1) - P.at(0, 1) * P.at(1, 0), 0.0f);
}
