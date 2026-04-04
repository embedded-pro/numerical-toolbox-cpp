#include "numerical/dynamics/RecursiveNewtonEuler.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    constexpr float gravity = 9.81f;
    const math::Vector<float, 3> gravityVec{ 0.0f, 0.0f, -gravity };

    // 1-DOF simple pendulum: single link rotating about z-axis
    // Link hangs along x-axis, joint at origin, CoM at (l/2, 0, 0)
    dynamics::RevoluteJointLink<float> MakePendulumLink(float mass, float length)
    {
        float I = mass * length * length / 12.0f; // thin rod about CoM

        return dynamics::RevoluteJointLink<float>{
            mass,
            math::SquareMatrix<float, 3>{
                { 0.0f, 0.0f, 0.0f },
                { 0.0f, I, 0.0f },
                { 0.0f, 0.0f, I } },
            math::Vector<float, 3>{ 0.0f, 0.0f, 1.0f },         // z-axis rotation
            math::Vector<float, 3>{},                           // joint at parent origin
            math::Vector<float, 3>{ length / 2.0f, 0.0f, 0.0f } // CoM at half-length
        };
    }

    // 2-DOF planar arm: two links rotating about z-axis in x-y plane
    std::array<dynamics::RevoluteJointLink<float>, 2> MakeTwoLinkArm(
        float m1, float l1, float m2, float l2)
    {
        float I1 = m1 * l1 * l1 / 12.0f;
        float I2 = m2 * l2 * l2 / 12.0f;

        dynamics::RevoluteJointLink<float> link1{
            m1,
            math::SquareMatrix<float, 3>{
                { 0.0f, 0.0f, 0.0f },
                { 0.0f, I1, 0.0f },
                { 0.0f, 0.0f, I1 } },
            math::Vector<float, 3>{ 0.0f, 0.0f, 1.0f },
            math::Vector<float, 3>{},
            math::Vector<float, 3>{ l1 / 2.0f, 0.0f, 0.0f }
        };

        dynamics::RevoluteJointLink<float> link2{
            m2,
            math::SquareMatrix<float, 3>{
                { 0.0f, 0.0f, 0.0f },
                { 0.0f, I2, 0.0f },
                { 0.0f, 0.0f, I2 } },
            math::Vector<float, 3>{ 0.0f, 0.0f, 1.0f },
            math::Vector<float, 3>{ l1, 0.0f, 0.0f }, // joint 2 at end of link 1
            math::Vector<float, 3>{ l2 / 2.0f, 0.0f, 0.0f }
        };

        return { link1, link2 };
    }

    class TestRecursiveNewtonEuler : public ::testing::Test
    {
    protected:
        dynamics::RecursiveNewtonEuler<float, 1> rnea1;
        dynamics::RecursiveNewtonEuler<float, 2> rnea2;
    };
}

TEST_F(TestRecursiveNewtonEuler, single_link_gravity_torque_at_horizontal)
{
    // Pendulum rotating about y-axis in x-z plane, horizontal at q=0 (link along x)
    float mass = 1.0f;
    float length = 1.0f;
    float I = mass * length * length / 12.0f;

    dynamics::RevoluteJointLink<float> link{
        mass,
        math::SquareMatrix<float, 3>{
            { 0.0f, 0.0f, 0.0f },
            { 0.0f, I, 0.0f },
            { 0.0f, 0.0f, I } },
        math::Vector<float, 3>{ 0.0f, 1.0f, 0.0f },         // y-axis rotation
        math::Vector<float, 3>{},                           // joint at parent origin
        math::Vector<float, 3>{ length / 2.0f, 0.0f, 0.0f } // CoM at half-length
    };
    std::array<dynamics::RevoluteJointLink<float>, 1> links = { link };

    math::Vector<float, 1> q{ 0.0f };
    math::Vector<float, 1> qDot{};
    math::Vector<float, 1> qDDot{};

    auto tau = rnea1.InverseDynamics(links, q, qDot, qDDot, gravityVec);

    // At q=0: link along x, gravity in -z, rotating about y-axis
    // Fictitious base acceleration = [0, 0, g], force on CoM = [0, 0, mg]
    // Torque = rCoM x force = [l/2, 0, 0] x [0, 0, mg] = [0, -mg*l/2, 0]
    // Projected on y-axis: tau = -m*g*l/2
    float expected = -mass * gravity * (length / 2.0f);
    EXPECT_NEAR(tau.at(0, 0), expected, 0.01f);
}

TEST_F(TestRecursiveNewtonEuler, single_link_zero_gravity_zero_motion_gives_zero_torque)
{
    auto link = MakePendulumLink(1.0f, 1.0f);
    std::array<dynamics::RevoluteJointLink<float>, 1> links = { link };

    math::Vector<float, 1> q{ 0.5f };
    math::Vector<float, 1> qDot{};
    math::Vector<float, 1> qDDot{};
    math::Vector<float, 3> zeroGravity{};

    auto tau = rnea1.InverseDynamics(links, q, qDot, qDDot, zeroGravity);

    // No gravity, no motion → zero torque
    EXPECT_NEAR(tau.at(0, 0), 0.0f, 1e-5f);
}

TEST_F(TestRecursiveNewtonEuler, single_link_pure_acceleration_no_gravity)
{
    float mass = 2.0f;
    float length = 1.0f;
    auto link = MakePendulumLink(mass, length);
    std::array<dynamics::RevoluteJointLink<float>, 1> links = { link };

    math::Vector<float, 1> q{ 0.0f };
    math::Vector<float, 1> qDot{};
    math::Vector<float, 1> qDDot{ 1.0f }; // 1 rad/s^2
    math::Vector<float, 3> zeroGravity{};

    auto tau = rnea1.InverseDynamics(links, q, qDot, qDDot, zeroGravity);

    // For a thin rod about the end: I_end = m*l^2/3
    // Torque = I_end * qDDot = 2 * 1 / 3 * 1 = 0.6667
    float I_end = mass * length * length / 3.0f;
    EXPECT_NEAR(tau.at(0, 0), I_end * 1.0f, 1e-3f);
}

TEST_F(TestRecursiveNewtonEuler, two_link_zero_gravity_zero_motion_gives_zero_torque)
{
    auto links = MakeTwoLinkArm(1.0f, 1.0f, 1.0f, 1.0f);

    math::Vector<float, 2> q{ 0.0f, 0.0f };
    math::Vector<float, 2> qDot{};
    math::Vector<float, 2> qDDot{};
    math::Vector<float, 3> zeroGravity{};

    auto tau = rnea2.InverseDynamics(links, q, qDot, qDDot, zeroGravity);

    EXPECT_NEAR(tau.at(0, 0), 0.0f, 1e-4f);
    EXPECT_NEAR(tau.at(1, 0), 0.0f, 1e-4f);
}

TEST_F(TestRecursiveNewtonEuler, two_link_joint2_acceleration_only)
{
    float m1 = 1.0f, l1 = 1.0f, m2 = 1.0f, l2 = 1.0f;
    auto links = MakeTwoLinkArm(m1, l1, m2, l2);

    math::Vector<float, 2> q{ 0.0f, 0.0f };
    math::Vector<float, 2> qDot{};
    math::Vector<float, 2> qDDot{ 0.0f, 1.0f }; // only joint 2 accelerates
    math::Vector<float, 3> zeroGravity{};

    auto tau = rnea2.InverseDynamics(links, q, qDot, qDDot, zeroGravity);

    // Joint 2 torque = I2_end * qDDot2 = m2*l2^2/3 * 1
    float I2_end = m2 * l2 * l2 / 3.0f;
    EXPECT_NEAR(tau.at(1, 0), I2_end, 1e-3f);

    // Joint 1 should also feel a reaction torque (coupling term)
    EXPECT_TRUE(std::abs(tau.at(0, 0)) > 1e-4f);
}

TEST_F(TestRecursiveNewtonEuler, single_link_centrifugal_term)
{
    // Link spinning at constant velocity → centrifugal force on CoM
    float mass = 1.0f;
    float length = 1.0f;
    auto link = MakePendulumLink(mass, length);
    std::array<dynamics::RevoluteJointLink<float>, 1> links = { link };

    math::Vector<float, 1> q{ 0.0f };
    math::Vector<float, 1> qDot{ 2.0f }; // spinning at 2 rad/s
    math::Vector<float, 1> qDDot{};      // constant velocity
    math::Vector<float, 3> zeroGravity{};

    auto tau = rnea1.InverseDynamics(links, q, qDot, qDDot, zeroGravity);

    // For a single link rotating in a plane, centrifugal force is radial
    // and doesn't produce torque about the rotation axis → tau ≈ 0
    EXPECT_NEAR(tau.at(0, 0), 0.0f, 1e-4f);
}

TEST_F(TestRecursiveNewtonEuler, two_link_symmetry_check)
{
    // Two identical links, both at q=0, both accelerating equally
    auto links = MakeTwoLinkArm(1.0f, 1.0f, 1.0f, 1.0f);

    math::Vector<float, 2> q{ 0.0f, 0.0f };
    math::Vector<float, 2> qDot{};
    math::Vector<float, 2> qDDot{ 1.0f, 1.0f };
    math::Vector<float, 3> zeroGravity{};

    auto tau = rnea2.InverseDynamics(links, q, qDot, qDDot, zeroGravity);

    // Joint 1 should require more torque than joint 2 (it carries the full chain)
    EXPECT_GT(std::abs(tau.at(0, 0)), std::abs(tau.at(1, 0)));
}
