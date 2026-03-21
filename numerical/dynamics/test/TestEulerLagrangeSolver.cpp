#include "numerical/dynamics/EulerLagrangeDynamics.hpp"
#include "numerical/dynamics/EulerLagrangeSolver.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    // Simple pendulum: 1-DOF
    //   M(q) = m * l^2
    //   C(q, qDot) = 0
    //   g(q) = m * gravity * l * sin(q)
    class SimplePendulum : public dynamics::EulerLagrangeDynamics<float, 1>
    {
    public:
        static constexpr float mass = 1.0f;
        static constexpr float length = 1.0f;
        static constexpr float gravity = 9.81f;

        MassMatrix ComputeMassMatrix(const StateVector& /* q */) const override
        {
            return MassMatrix{ mass * length * length };
        }

        StateVector ComputeCoriolisTerms(const StateVector& /* q */, const StateVector& /* qDot */) const override
        {
            return StateVector{};
        }

        StateVector ComputeGravityTerms(const StateVector& q) const override
        {
            return StateVector{ mass * gravity * length * std::sin(q.at(0, 0)) };
        }
    };

    // Two-link planar arm: 2-DOF
    //   m1, m2: link masses
    //   l1, l2: link lengths
    //   lc1, lc2: center-of-mass distances
    //   I1, I2: link inertias
    class TwoLinkPlanarArm : public dynamics::EulerLagrangeDynamics<float, 2>
    {
    public:
        static constexpr float m1 = 1.0f;
        static constexpr float m2 = 1.0f;
        static constexpr float l1 = 1.0f;
        static constexpr float lc1 = 0.5f;
        static constexpr float lc2 = 0.5f;
        static constexpr float I1 = 0.083f; // m1 * l1^2 / 12
        static constexpr float I2 = 0.083f;
        static constexpr float gravity = 9.81f;

        MassMatrix ComputeMassMatrix(const StateVector& q) const override
        {
            float c2 = std::cos(q.at(1, 0));

            float d11 = I1 + I2 + m1 * lc1 * lc1 + m2 * (l1 * l1 + lc2 * lc2 + 2.0f * l1 * lc2 * c2);
            float d12 = I2 + m2 * (lc2 * lc2 + l1 * lc2 * c2);
            float d22 = I2 + m2 * lc2 * lc2;

            return MassMatrix{
                { d11, d12 },
                { d12, d22 }
            };
        }

        StateVector ComputeCoriolisTerms(const StateVector& q, const StateVector& qDot) const override
        {
            float s2 = std::sin(q.at(1, 0));
            float h = m2 * l1 * lc2 * s2;

            float c1 = -h * qDot.at(1, 0) * (2.0f * qDot.at(0, 0) + qDot.at(1, 0));
            float c2Term = h * qDot.at(0, 0) * qDot.at(0, 0);

            return StateVector{ c1, c2Term };
        }

        StateVector ComputeGravityTerms(const StateVector& q) const override
        {
            float s1 = std::sin(q.at(0, 0));
            float s12 = std::sin(q.at(0, 0) + q.at(1, 0));

            float g1 = (m1 * lc1 + m2 * l1) * gravity * s1 + m2 * lc2 * gravity * s12;
            float g2 = m2 * lc2 * gravity * s12;

            return StateVector{ g1, g2 };
        }
    };

    class TestEulerLagrangeSolver : public ::testing::Test
    {
    protected:
        dynamics::EulerLagrangeSolver<float, 1> solver1Dof;
        dynamics::EulerLagrangeSolver<float, 2> solver2Dof;
        SimplePendulum pendulum;
        TwoLinkPlanarArm twoLinkArm;
    };
}

TEST_F(TestEulerLagrangeSolver, forward_dynamics_at_rest_returns_gravity_acceleration)
{
    math::Vector<float, 1> q{ 0.5f }; // 0.5 rad from vertical
    math::Vector<float, 1> qDot{};
    math::Vector<float, 1> tau{};

    auto qDDot = solver1Dof.ForwardDynamics(pendulum, q, qDot, tau);

    // qDDot = -g * sin(q) / l = -9.81 * sin(0.5)
    float expected = -SimplePendulum::gravity * std::sin(0.5f) / SimplePendulum::length;
    EXPECT_NEAR(qDDot.at(0, 0), expected, 1e-4f);
}

TEST_F(TestEulerLagrangeSolver, inverse_dynamics_at_rest_returns_gravity_compensation)
{
    math::Vector<float, 1> q{ 0.5f };
    math::Vector<float, 1> qDot{};
    math::Vector<float, 1> qDDot{};

    auto tau = solver1Dof.InverseDynamics(pendulum, q, qDot, qDDot);

    // tau = g(q) = m * gravity * l * sin(q)
    float expected = SimplePendulum::mass * SimplePendulum::gravity * SimplePendulum::length * std::sin(0.5f);
    EXPECT_NEAR(tau.at(0, 0), expected, 1e-4f);
}

TEST_F(TestEulerLagrangeSolver, forward_inverse_roundtrip_consistency)
{
    math::Vector<float, 1> q{ 1.0f };
    math::Vector<float, 1> qDot{ 0.3f };
    math::Vector<float, 1> tau{ 2.5f };

    auto qDDot = solver1Dof.ForwardDynamics(pendulum, q, qDot, tau);
    auto tauRecovered = solver1Dof.InverseDynamics(pendulum, q, qDot, qDDot);

    EXPECT_NEAR(tauRecovered.at(0, 0), tau.at(0, 0), 1e-4f);
}

TEST_F(TestEulerLagrangeSolver, identity_mass_matrix_forward_dynamics_reduces_to_subtraction)
{
    // A trivial model: M=I, C=0, g=0
    class TrivialModel : public dynamics::EulerLagrangeDynamics<float, 2>
    {
    public:
        MassMatrix ComputeMassMatrix(const StateVector& /* q */) const override
        {
            return MassMatrix::Identity();
        }

        StateVector ComputeCoriolisTerms(const StateVector& /* q */, const StateVector& /* qDot */) const override
        {
            return StateVector{};
        }

        StateVector ComputeGravityTerms(const StateVector& /* q */) const override
        {
            return StateVector{};
        }
    };

    TrivialModel model;
    math::Vector<float, 2> q{ 1.0f, 2.0f };
    math::Vector<float, 2> qDot{ 0.5f, 0.5f };
    math::Vector<float, 2> tau{ 3.0f, 4.0f };

    auto qDDot = solver2Dof.ForwardDynamics(model, q, qDot, tau);

    EXPECT_NEAR(qDDot.at(0, 0), tau.at(0, 0), 1e-3f);
    EXPECT_NEAR(qDDot.at(1, 0), tau.at(1, 0), 1e-3f);
}

TEST_F(TestEulerLagrangeSolver, two_link_forward_inverse_roundtrip)
{
    math::Vector<float, 2> q{ 0.3f, 0.7f };
    math::Vector<float, 2> qDot{ 0.1f, -0.2f };
    math::Vector<float, 2> tau{ 1.5f, 0.8f };

    auto qDDot = solver2Dof.ForwardDynamics(twoLinkArm, q, qDot, tau);
    auto tauRecovered = solver2Dof.InverseDynamics(twoLinkArm, q, qDot, qDDot);

    EXPECT_NEAR(tauRecovered.at(0, 0), tau.at(0, 0), 1e-3f);
    EXPECT_NEAR(tauRecovered.at(1, 0), tau.at(1, 0), 1e-3f);
}

TEST_F(TestEulerLagrangeSolver, two_link_at_rest_returns_gravity_torques)
{
    math::Vector<float, 2> q{ 0.0f, 0.0f }; // hanging straight down
    math::Vector<float, 2> qDot{};
    math::Vector<float, 2> tau{};

    auto qDDot = solver2Dof.ForwardDynamics(twoLinkArm, q, qDot, tau);

    // At q = [0, 0], sin(q1)=0, sin(q1+q2)=0 → g=[0,0] → qDDot=[0,0]
    EXPECT_NEAR(qDDot.at(0, 0), 0.0f, 1e-4f);
    EXPECT_NEAR(qDDot.at(1, 0), 0.0f, 1e-4f);
}

TEST_F(TestEulerLagrangeSolver, inverse_dynamics_zero_acceleration_returns_only_coriolis_and_gravity)
{
    math::Vector<float, 2> q{ 0.5f, 0.3f };
    math::Vector<float, 2> qDot{ 1.0f, -0.5f };
    math::Vector<float, 2> qDDot{};

    auto tau = solver2Dof.InverseDynamics(twoLinkArm, q, qDot, qDDot);

    auto C = twoLinkArm.ComputeCoriolisTerms(q, qDot);
    auto g = twoLinkArm.ComputeGravityTerms(q);

    EXPECT_NEAR(tau.at(0, 0), C.at(0, 0) + g.at(0, 0), 1e-4f);
    EXPECT_NEAR(tau.at(1, 0), C.at(1, 0) + g.at(1, 0), 1e-4f);
}
