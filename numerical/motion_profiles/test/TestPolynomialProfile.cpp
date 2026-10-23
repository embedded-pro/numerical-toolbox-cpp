#include "numerical/motion_profiles/PolynomialProfile.hpp"
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    using motion_profiles::BoundaryConditions;
    using motion_profiles::Degree;
    using motion_profiles::PolynomialProfile;

    class TestPolynomialProfile : public ::testing::Test
    {
    protected:
        static constexpr float tolerance{ 1e-5f };
        const BoundaryConditions<float> restToRest{ 0.0f, 1.0f };
        PolynomialProfile<float> cubic{ restToRest, 2.0f, Degree::Cubic };
    };
}

TEST_F(TestPolynomialProfile, endpoints_match_boundary_positions)
{
    EXPECT_NEAR(cubic.Sample(0.0f).position, 0.0f, tolerance);
    EXPECT_NEAR(cubic.Sample(cubic.Duration()).position, 1.0f, tolerance);
}

TEST_F(TestPolynomialProfile, endpoint_velocities_are_matched)
{
    EXPECT_NEAR(cubic.Sample(0.0f).velocity, 0.0f, tolerance);
    EXPECT_NEAR(cubic.Sample(cubic.Duration()).velocity, 0.0f, tolerance);
}

TEST_F(TestPolynomialProfile, cubic_midpoint_is_symmetric)
{
    EXPECT_NEAR(cubic.Sample(1.0f).position, 0.5f, tolerance);
}

TEST_F(TestPolynomialProfile, cubic_peak_velocity_matches_formula)
{
    float peak{ 0.0f };
    for (int i = 0; i <= 200; ++i)
        peak = std::max(peak, std::abs(cubic.Sample(0.01f * static_cast<float>(i)).velocity));

    EXPECT_NEAR(peak, 1.5f * 1.0f / 2.0f, tolerance);
    EXPECT_NEAR(cubic.Sample(1.0f).velocity, 0.75f, tolerance);
}

TEST_F(TestPolynomialProfile, cubic_jerk_is_constant_third_derivative)
{
    for (float t : { 0.0f, 1.0f, 2.0f })
        EXPECT_NEAR(cubic.Sample(t).jerk, -1.5f, tolerance);
}

TEST_F(TestPolynomialProfile, quintic_endpoint_accelerations_are_zero)
{
    const PolynomialProfile<float> quintic{ restToRest, 2.0f, Degree::Quintic };

    EXPECT_NEAR(quintic.Sample(0.0f).acceleration, 0.0f, tolerance);
    EXPECT_NEAR(quintic.Sample(2.0f).acceleration, 0.0f, tolerance);
    EXPECT_NEAR(quintic.Sample(2.0f).position, 1.0f, tolerance);
    EXPECT_NEAR(quintic.Sample(2.0f).velocity, 0.0f, tolerance);
}

TEST_F(TestPolynomialProfile, nonzero_boundary_velocity_is_honoured)
{
    const PolynomialProfile<float> moving{ BoundaryConditions<float>{ 0.0f, 1.0f, 0.5f, -0.5f }, 2.0f, Degree::Cubic };

    EXPECT_NEAR(moving.Sample(0.0f).velocity, 0.5f, tolerance);
    EXPECT_NEAR(moving.Sample(2.0f).velocity, -0.5f, tolerance);
}

TEST_F(TestPolynomialProfile, sample_clamps_outside_domain)
{
    EXPECT_EQ(cubic.Sample(-1.0f), cubic.Sample(0.0f));
    EXPECT_EQ(cubic.Sample(cubic.Duration() + 1.0f), cubic.Sample(cubic.Duration()));
}

TEST_F(TestPolynomialProfile, reset_recomputes_coefficients)
{
    cubic.Reset(BoundaryConditions<float>{ 1.0f, 3.0f }, 1.0f);

    EXPECT_EQ(cubic.Duration(), 1.0f);
    EXPECT_NEAR(cubic.Sample(0.0f).position, 1.0f, tolerance);
    EXPECT_NEAR(cubic.Sample(1.0f).position, 3.0f, tolerance);
}
