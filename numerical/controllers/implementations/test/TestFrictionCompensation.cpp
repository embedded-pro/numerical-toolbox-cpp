#include "numerical/controllers/implementations/FrictionCompensation.hpp"
#include <array>
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    using Parameters = controllers::FrictionParameters<float>;
    using Axis = controllers::FrictionCompensation<float, 1>;
    using ThreeAxes = controllers::FrictionCompensation<float, 3>;

    class TestFrictionCompensation : public ::testing::Test
    {
    protected:
        Parameters p{ 0.5f, 0.8f, 0.1f, 0.05f, 2.0f, 1e-3f, 10.0f };
        Axis friction{ { p } };

        float Torque(float velocity) const
        {
            return friction.Compute(Axis::AxisVector{ velocity })[0];
        }
    };
}

TEST_F(TestFrictionCompensation, positive_velocity_positive_torque)
{
    const float tau{ Torque(1.0f) };

    EXPECT_GT(tau, 0.0f);
    EXPECT_NEAR(tau, p.coulomb + p.viscous * 1.0f, 1e-6f);
}

TEST_F(TestFrictionCompensation, sign_flips_with_velocity)
{
    for (float velocity : { 1e-4f, 0.03f, 0.2f, 2.0f })
        EXPECT_FLOAT_EQ(Torque(velocity), -Torque(-velocity));
}

TEST_F(TestFrictionCompensation, viscous_dominates_at_high_speed)
{
    for (float velocity : { 2.0f, 4.0f, 6.0f })
        EXPECT_NEAR(Torque(velocity + 2.0f) - Torque(velocity), p.viscous * 2.0f, 1e-5f);
}

TEST_F(TestFrictionCompensation, stiction_exceeds_coulomb_near_zero)
{
    const float velocity{ 10.0f * p.smoothingVelocity };
    const float fall{ std::exp(-std::pow(velocity / p.stribeckVelocity, p.stribeckShape)) };
    const float level{ p.coulomb + (p.stiction - p.coulomb) * fall };

    const float stribeck{ Axis::AxisTorque(p, velocity) - p.viscous * velocity };

    EXPECT_NEAR(stribeck, 0.788237f, 1e-5f);
    EXPECT_NEAR(stribeck, level * std::tanh(10.0f), 1e-6f);
    EXPECT_GT(stribeck, p.coulomb);
}

TEST_F(TestFrictionCompensation, stribeck_decays_to_coulomb)
{
    const float velocity{ 10.0f * p.stribeckVelocity };

    EXPECT_NEAR(Axis::AxisTorque(p, velocity) - p.viscous * velocity, p.coulomb, 1e-6f);
}

TEST_F(TestFrictionCompensation, smooth_through_zero)
{
    EXPECT_EQ(Torque(0.0f), 0.0f);
    EXPECT_NEAR(Torque(1e-6f), 0.0f, 1e-3f);
    EXPECT_NEAR(Torque(-1e-6f), 0.0f, 1e-3f);
}

TEST_F(TestFrictionCompensation, multi_axis_elementwise)
{
    const ThreeAxes::Parameters params{ p,
        Parameters{ 0.2f, 0.2f, 0.05f, 0.1f, 1.0f, 1e-2f, 5.0f },
        Parameters{ 1.0f, 1.5f, 0.3f, 0.02f, 2.0f, 1e-3f, 5.0f } };
    const ThreeAxes friction3{ params };
    const ThreeAxes::AxisVector velocity{ 0.02f, -0.3f, 1.5f };

    const auto tau{ friction3.Compute(velocity) };

    for (std::size_t i = 0; i < 3; ++i)
        EXPECT_FLOAT_EQ(tau[i], ThreeAxes::AxisTorque(params[i], velocity[i]));
}

TEST_F(TestFrictionCompensation, feedforward_additive_cancels_model)
{
    constexpr float controlTorque{ 0.25f };

    for (float velocity : { -0.5f, -0.01f, 0.03f, 0.7f })
    {
        const float command{ controlTorque + Torque(velocity) };
        const float plantFriction{ -Axis::AxisTorque(p, velocity) };

        EXPECT_NEAR(command + plantFriction, controlTorque, 1e-6f);
    }
}

TEST_F(TestFrictionCompensation, output_is_clamped_to_max_compensation)
{
    Parameters limited{ p };
    limited.maxCompensation = 0.6f;
    const Axis clamped{ { limited } };

    EXPECT_EQ(clamped.Compute(Axis::AxisVector{ 5.0f })[0], 0.6f);
    EXPECT_EQ(clamped.Compute(Axis::AxisVector{ -5.0f })[0], -0.6f);
}
