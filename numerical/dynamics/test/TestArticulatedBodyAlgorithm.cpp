#include "numerical/dynamics/ArticulatedBodyAlgorithm.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    constexpr float gravity = 9.81f;
    const math::Vector<float, 3> gravityVec{ 0.0f, 0.0f, -gravity };

    dynamics::RevoluteJointLink<float> MakePendulumLink(float mass, float length)
    {
        float I = mass * length * length / 12.0f;

        return dynamics::RevoluteJointLink<float>{
            mass,
            math::SquareMatrix<float, 3>{
                { 0.0f, 0.0f, 0.0f },
                { 0.0f, I, 0.0f },
                { 0.0f, 0.0f, I } },
            math::Vector<float, 3>{ 0.0f, 0.0f, 1.0f },
            math::Vector<float, 3>{},
            math::Vector<float, 3>{ length / 2.0f, 0.0f, 0.0f }
        };
    }

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
            math::Vector<float, 3>{ l1, 0.0f, 0.0f },
            math::Vector<float, 3>{ l2 / 2.0f, 0.0f, 0.0f }
        };

        return { link1, link2 };
    }

    class TestArticulatedBodyAlgorithm : public ::testing::Test
    {
    protected:
        dynamics::ArticulatedBodyAlgorithm<float, 1> aba1;
        dynamics::ArticulatedBodyAlgorithm<float, 2> aba2;
    };
}

TEST_F(TestArticulatedBodyAlgorithm, single_link_zero_torque_no_gravity_gives_zero_acceleration)
{
    auto link = MakePendulumLink(1.0f, 1.0f);
    std::array<dynamics::RevoluteJointLink<float>, 1> links = { link };

    math::Vector<float, 1> q{ 0.0f };
    math::Vector<float, 1> qDot{};
    math::Vector<float, 1> tau{};
    math::Vector<float, 3> zeroGravity{};

    auto qDDot = aba1.ForwardDynamics(links, q, qDot, tau, zeroGravity);

    EXPECT_NEAR(qDDot.at(0, 0), 0.0f, 1e-5f);
}

TEST_F(TestArticulatedBodyAlgorithm, single_link_pure_torque_no_gravity)
{
    float mass = 2.0f;
    float length = 1.0f;
    auto link = MakePendulumLink(mass, length);
    std::array<dynamics::RevoluteJointLink<float>, 1> links = { link };

    math::Vector<float, 1> q{ 0.0f };
    math::Vector<float, 1> qDot{};
    math::Vector<float, 3> zeroGravity{};

    // Apply torque = I_end to get qDDot = 1 rad/s^2
    float I_end = mass * length * length / 3.0f;
    math::Vector<float, 1> tau{ I_end };

    auto qDDot = aba1.ForwardDynamics(links, q, qDot, tau, zeroGravity);

    EXPECT_NEAR(qDDot.at(0, 0), 1.0f, 1e-3f);
}

TEST_F(TestArticulatedBodyAlgorithm, single_link_rnea_aba_roundtrip_no_gravity)
{
    auto link = MakePendulumLink(2.0f, 1.0f);
    std::array<dynamics::RevoluteJointLink<float>, 1> links = { link };

    math::Vector<float, 1> q{ 0.5f };
    math::Vector<float, 1> qDot{ 1.5f };
    math::Vector<float, 1> qDDotExpected{ 3.0f };
    math::Vector<float, 3> zeroGravity{};

    math::Vector<float, 1> tau{ 2.0f };

    auto qDDotRecovered = aba1.ForwardDynamics(links, q, qDot, tau, zeroGravity);

    EXPECT_NEAR(qDDotRecovered.at(0, 0), qDDotExpected.at(0, 0), 1e-3f);
}

TEST_F(TestArticulatedBodyAlgorithm, single_link_rnea_aba_roundtrip_with_gravity)
{
    auto link = MakePendulumLink(2.0f, 1.0f);
    std::array<dynamics::RevoluteJointLink<float>, 1> links = { link };

    math::Vector<float, 1> q{ 0.3f };
    math::Vector<float, 1> qDot{ 0.0f };
    math::Vector<float, 1> qDDotExpected{ 2.0f };

    math::Vector<float, 1> tau{ 1.3333333731f };
    auto qDDotRecovered = aba1.ForwardDynamics(links, q, qDot, tau, gravityVec);

    EXPECT_NEAR(qDDotRecovered.at(0, 0), qDDotExpected.at(0, 0), 1e-2f);
}

TEST_F(TestArticulatedBodyAlgorithm, two_link_zero_torque_no_gravity_gives_zero_acceleration)
{
    auto links = MakeTwoLinkArm(1.0f, 1.0f, 1.0f, 1.0f);

    math::Vector<float, 2> q{ 0.0f, 0.0f };
    math::Vector<float, 2> qDot{};
    math::Vector<float, 2> tau{};
    math::Vector<float, 3> zeroGravity{};

    auto qDDot = aba2.ForwardDynamics(links, q, qDot, tau, zeroGravity);

    EXPECT_NEAR(qDDot.at(0, 0), 0.0f, 1e-4f);
    EXPECT_NEAR(qDDot.at(1, 0), 0.0f, 1e-4f);
}

TEST_F(TestArticulatedBodyAlgorithm, two_link_analytical_torque_produces_known_acceleration)
{
    // Textbook 2R planar arm (z-axis joints, uniform rods):
    //   M(q) = | m1*l1^2/3 + m2*(l1^2 + l2^2/3 + l1*l2*cos(q2))    m2*(l2^2/3 + l1*l2*cos(q2)/2) |
    //          | m2*(l2^2/3 + l1*l2*cos(q2)/2)                        m2*l2^2/3                     |
    //   C(q,qd)*qd_0 = h*(2*qd1*qd2 + qd2^2)/2,  h = -m2*l1*l2*sin(q2)
    //   C(q,qd)*qd_1 = -h*qd1^2/2
    //   G = 0 (z-axis rotation with z-gravity => no gravitational torque)
    //
    // tau = M*qDDot + C*qDot  =>  qDDot = M^{-1}*(tau - C*qDot)

    float m1 = 1.0f, l1 = 1.0f, m2 = 1.0f, l2 = 1.0f;
    float q2 = -0.5f;
    float qd1 = 1.0f, qd2 = -0.5f;
    float qdd1 = 2.0f, qdd2 = -1.0f;

    float c2 = std::cos(q2);
    float s2 = std::sin(q2);
    float h = -m2 * l1 * l2 * s2;

    float M00 = m1 * l1 * l1 / 3.0f + m2 * (l1 * l1 + l2 * l2 / 3.0f + l1 * l2 * c2);
    float M01 = m2 * (l2 * l2 / 3.0f + l1 * l2 * c2 / 2.0f);
    float M11 = m2 * l2 * l2 / 3.0f;

    float Cqd0 = h * (2.0f * qd1 * qd2 + qd2 * qd2) / 2.0f;
    float Cqd1 = -h * qd1 * qd1 / 2.0f;

    float tau0 = M00 * qdd1 + M01 * qdd2 + Cqd0;
    float tau1 = M01 * qdd1 + M11 * qdd2 + Cqd1;

    auto links = MakeTwoLinkArm(m1, l1, m2, l2);
    math::Vector<float, 2> q{ 0.3f, q2 };
    math::Vector<float, 2> qDot{ qd1, qd2 };
    math::Vector<float, 2> tau{ tau0, tau1 };
    math::Vector<float, 3> zeroGravity{};

    auto qDDot = aba2.ForwardDynamics(links, q, qDot, tau, zeroGravity);

    EXPECT_NEAR(qDDot.at(0, 0), qdd1, 1e-4f);
    EXPECT_NEAR(qDDot.at(1, 0), qdd2, 1e-4f);
}

TEST_F(TestArticulatedBodyAlgorithm, two_link_coriolis_torque_gives_zero_acceleration)
{
    // If tau = C(q,qDot)*qDot exactly, then M*qDDot = 0 => qDDot = 0
    float m1 = 1.0f, l1 = 1.0f, m2 = 1.0f, l2 = 1.0f;
    float q2 = -0.5f;
    float qd1 = 1.0f, qd2 = -0.5f;

    float h = -m2 * l1 * l2 * std::sin(q2);
    float tau0 = h * (2.0f * qd1 * qd2 + qd2 * qd2) / 2.0f;
    float tau1 = -h * qd1 * qd1 / 2.0f;

    auto links = MakeTwoLinkArm(m1, l1, m2, l2);
    math::Vector<float, 2> q{ 0.3f, q2 };
    math::Vector<float, 2> qDot{ qd1, qd2 };
    math::Vector<float, 2> tau{ tau0, tau1 };
    math::Vector<float, 3> zeroGravity{};

    auto qDDot = aba2.ForwardDynamics(links, q, qDot, tau, zeroGravity);

    EXPECT_NEAR(qDDot.at(0, 0), 0.0f, 1e-5f);
    EXPECT_NEAR(qDDot.at(1, 0), 0.0f, 1e-5f);
}

TEST_F(TestArticulatedBodyAlgorithm, two_link_rnea_aba_roundtrip_no_gravity)
{
    auto links = MakeTwoLinkArm(1.0f, 1.0f, 1.0f, 1.0f);

    math::Vector<float, 2> q{ 0.3f, -0.5f };
    math::Vector<float, 2> qDot{ 1.0f, -0.5f };
    math::Vector<float, 2> qDDotExpected{ 2.0f, -1.0f };
    math::Vector<float, 3> zeroGravity{};

    math::Vector<float, 2> tau{ 4.1365890503f, 0.9712030888f };
    auto qDDotRecovered = aba2.ForwardDynamics(links, q, qDot, tau, zeroGravity);

    EXPECT_NEAR(qDDotRecovered.at(0, 0), qDDotExpected.at(0, 0), 1e-4f);
    EXPECT_NEAR(qDDotRecovered.at(1, 0), qDDotExpected.at(1, 0), 1e-4f);
}

TEST_F(TestArticulatedBodyAlgorithm, two_link_rnea_aba_roundtrip_with_gravity)
{
    auto links = MakeTwoLinkArm(2.0f, 0.5f, 1.0f, 0.8f);

    math::Vector<float, 2> q{ 0.1f, -0.2f };
    math::Vector<float, 2> qDot{ 0.5f, -0.3f };
    math::Vector<float, 2> qDDotExpected{ 1.0f, -2.0f };

    math::Vector<float, 2> tau{ 0.1949892044f, -0.0272534918f };
    auto qDDotRecovered = aba2.ForwardDynamics(links, q, qDot, tau, gravityVec);

    EXPECT_NEAR(qDDotRecovered.at(0, 0), qDDotExpected.at(0, 0), 1e-4f);
    EXPECT_NEAR(qDDotRecovered.at(1, 0), qDDotExpected.at(1, 0), 1e-4f);
}

TEST_F(TestArticulatedBodyAlgorithm, single_link_free_fall_under_gravity)
{
    // y-axis pendulum at q=0: joint axis = y, CoM at [L/2, 0, 0].
    // Gravity = [0, 0, -g]. At q=0, link frame = world frame.
    // Potential energy: V = -mgL*sin(q)/2  (positive y-rotation moves CoM toward -z)
    // G(q) = dV/dq = -mgL*cos(q)/2
    // At q=0: G = -mgL/2
    // EoM: I_pivot * qDDot + G = 0  =>  qDDot = -G / I_pivot = +mgL / (2*I_pivot)
    // I_pivot = mL^2/3
    // qDDot = +3g/(2L)
    float mass = 1.0f;
    float length = 1.0f;
    float I = mass * length * length / 12.0f;

    dynamics::RevoluteJointLink<float> link{
        mass,
        math::SquareMatrix<float, 3>{
            { 0.0f, 0.0f, 0.0f },
            { 0.0f, I, 0.0f },
            { 0.0f, 0.0f, I } },
        math::Vector<float, 3>{ 0.0f, 1.0f, 0.0f },
        math::Vector<float, 3>{},
        math::Vector<float, 3>{ length / 2.0f, 0.0f, 0.0f }
    };
    std::array<dynamics::RevoluteJointLink<float>, 1> links = { link };

    math::Vector<float, 1> q{ 0.0f };
    math::Vector<float, 1> qDot{};
    math::Vector<float, 1> tau{};

    auto qDDot = aba1.ForwardDynamics(links, q, qDot, tau, gravityVec);

    float expected = 3.0f * gravity / (2.0f * length);
    EXPECT_NEAR(qDDot.at(0, 0), expected, 1e-4f);
}
