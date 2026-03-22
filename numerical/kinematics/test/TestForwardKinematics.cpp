#include "numerical/kinematics/ForwardKinematics.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    constexpr float tolerance = 5e-4f;
    constexpr float pi = std::numbers::pi_v<float>;

    dynamics::RevoluteJointLink<float> MakeLink(float mass, float length,
        const math::Vector<float, 3>& axis,
        const math::Vector<float, 3>& parentToJoint,
        const math::Vector<float, 3>& jointToCoM)
    {
        float I = mass * length * length / 12.0f;

        return dynamics::RevoluteJointLink<float>{
            mass,
            math::SquareMatrix<float, 3>{
                { I, 0.0f, 0.0f },
                { 0.0f, I, 0.0f },
                { 0.0f, 0.0f, I } },
            axis,
            parentToJoint,
            jointToCoM
        };
    }

    dynamics::RevoluteJointLink<float> MakePlanarLink(float mass, float length,
        const math::Vector<float, 3>& parentToJoint)
    {
        return MakeLink(mass, length,
            math::Vector<float, 3>{ 0.0f, 0.0f, 1.0f },
            parentToJoint,
            math::Vector<float, 3>{ length / 2.0f, 0.0f, 0.0f });
    }

    class TestForwardKinematics : public ::testing::Test
    {
    protected:
        kinematics::ForwardKinematics<float, 1> fk1;
        kinematics::ForwardKinematics<float, 2> fk2;
        kinematics::ForwardKinematics<float, 3> fk3;
    };
}

TEST_F(TestForwardKinematics, single_link_at_zero_angle)
{
    float length = 1.0f;
    auto link = MakePlanarLink(1.0f, length, math::Vector<float, 3>{});
    std::array<dynamics::RevoluteJointLink<float>, 1> links = { link };

    math::Vector<float, 1> q{ 0.0f };
    auto positions = fk1.Compute(links, q);

    EXPECT_NEAR(positions[0].at(0, 0), 0.0f, tolerance);
    EXPECT_NEAR(positions[0].at(1, 0), 0.0f, tolerance);
    EXPECT_NEAR(positions[0].at(2, 0), 0.0f, tolerance);

    EXPECT_NEAR(positions[1].at(0, 0), length, tolerance);
    EXPECT_NEAR(positions[1].at(1, 0), 0.0f, tolerance);
    EXPECT_NEAR(positions[1].at(2, 0), 0.0f, tolerance);
}

TEST_F(TestForwardKinematics, single_link_rotated_90_degrees)
{
    float length = 1.0f;
    auto link = MakePlanarLink(1.0f, length, math::Vector<float, 3>{});
    std::array<dynamics::RevoluteJointLink<float>, 1> links = { link };

    math::Vector<float, 1> q{ pi / 2.0f };
    auto positions = fk1.Compute(links, q);

    EXPECT_NEAR(positions[1].at(0, 0), 0.0f, tolerance);
    EXPECT_NEAR(positions[1].at(1, 0), length, tolerance);
    EXPECT_NEAR(positions[1].at(2, 0), 0.0f, tolerance);
}

TEST_F(TestForwardKinematics, single_link_rotated_180_degrees)
{
    float length = 2.0f;
    auto link = MakePlanarLink(1.0f, length, math::Vector<float, 3>{});
    std::array<dynamics::RevoluteJointLink<float>, 1> links = { link };

    math::Vector<float, 1> q{ pi };
    auto positions = fk1.Compute(links, q);

    EXPECT_NEAR(positions[1].at(0, 0), -length, tolerance);
    EXPECT_NEAR(positions[1].at(1, 0), 0.0f, tolerance);
    EXPECT_NEAR(positions[1].at(2, 0), 0.0f, tolerance);
}

TEST_F(TestForwardKinematics, two_link_planar_both_zero)
{
    float l1 = 1.0f, l2 = 0.5f;

    auto link1 = MakePlanarLink(1.0f, l1, math::Vector<float, 3>{});
    auto link2 = MakePlanarLink(0.5f, l2, math::Vector<float, 3>{ l1, 0.0f, 0.0f });
    std::array<dynamics::RevoluteJointLink<float>, 2> links = { link1, link2 };

    math::Vector<float, 2> q{ 0.0f, 0.0f };
    auto positions = fk2.Compute(links, q);

    EXPECT_NEAR(positions[0].at(0, 0), 0.0f, tolerance);
    EXPECT_NEAR(positions[1].at(0, 0), l1, tolerance);
    EXPECT_NEAR(positions[1].at(1, 0), 0.0f, tolerance);
    EXPECT_NEAR(positions[2].at(0, 0), l1 + l2, tolerance);
    EXPECT_NEAR(positions[2].at(1, 0), 0.0f, tolerance);
}

TEST_F(TestForwardKinematics, two_link_planar_first_joint_90)
{
    float l1 = 1.0f, l2 = 1.0f;

    auto link1 = MakePlanarLink(1.0f, l1, math::Vector<float, 3>{});
    auto link2 = MakePlanarLink(1.0f, l2, math::Vector<float, 3>{ l1, 0.0f, 0.0f });
    std::array<dynamics::RevoluteJointLink<float>, 2> links = { link1, link2 };

    math::Vector<float, 2> q{ pi / 2.0f, 0.0f };
    auto positions = fk2.Compute(links, q);

    EXPECT_NEAR(positions[1].at(0, 0), 0.0f, tolerance);
    EXPECT_NEAR(positions[1].at(1, 0), l1, tolerance);

    EXPECT_NEAR(positions[2].at(0, 0), 0.0f, tolerance);
    EXPECT_NEAR(positions[2].at(1, 0), l1 + l2, tolerance);
}

TEST_F(TestForwardKinematics, two_link_planar_second_joint_90)
{
    float l1 = 1.0f, l2 = 1.0f;

    auto link1 = MakePlanarLink(1.0f, l1, math::Vector<float, 3>{});
    auto link2 = MakePlanarLink(1.0f, l2, math::Vector<float, 3>{ l1, 0.0f, 0.0f });
    std::array<dynamics::RevoluteJointLink<float>, 2> links = { link1, link2 };

    math::Vector<float, 2> q{ 0.0f, pi / 2.0f };
    auto positions = fk2.Compute(links, q);

    EXPECT_NEAR(positions[1].at(0, 0), l1, tolerance);
    EXPECT_NEAR(positions[1].at(1, 0), 0.0f, tolerance);

    EXPECT_NEAR(positions[2].at(0, 0), l1, tolerance);
    EXPECT_NEAR(positions[2].at(1, 0), l2, tolerance);
}

TEST_F(TestForwardKinematics, two_link_folded_back)
{
    float l1 = 1.0f, l2 = 1.0f;

    auto link1 = MakePlanarLink(1.0f, l1, math::Vector<float, 3>{});
    auto link2 = MakePlanarLink(1.0f, l2, math::Vector<float, 3>{ l1, 0.0f, 0.0f });
    std::array<dynamics::RevoluteJointLink<float>, 2> links = { link1, link2 };

    math::Vector<float, 2> q{ 0.0f, pi };
    auto positions = fk2.Compute(links, q);

    EXPECT_NEAR(positions[1].at(0, 0), l1, tolerance);
    EXPECT_NEAR(positions[2].at(0, 0), l1 - l2, tolerance);
    EXPECT_NEAR(positions[2].at(1, 0), 0.0f, tolerance);
}

TEST_F(TestForwardKinematics, y_axis_rotation_single_link)
{
    float length = 1.0f;
    auto link = MakeLink(1.0f, length,
        math::Vector<float, 3>{ 0.0f, 1.0f, 0.0f },
        math::Vector<float, 3>{},
        math::Vector<float, 3>{ length / 2.0f, 0.0f, 0.0f });
    std::array<dynamics::RevoluteJointLink<float>, 1> links = { link };

    math::Vector<float, 1> q{ pi / 2.0f };
    auto positions = fk1.Compute(links, q);

    EXPECT_NEAR(positions[1].at(1, 0), 0.0f, tolerance);
    EXPECT_NEAR(std::sqrt(positions[1].at(0, 0) * positions[1].at(0, 0) +
                          positions[1].at(2, 0) * positions[1].at(2, 0)),
        length, tolerance);
}

TEST_F(TestForwardKinematics, base_position_always_origin)
{
    float length = 1.5f;
    auto link = MakePlanarLink(2.0f, length, math::Vector<float, 3>{});
    std::array<dynamics::RevoluteJointLink<float>, 1> links = { link };

    for (float angle : { 0.0f, pi / 4.0f, pi / 2.0f, pi, -pi / 3.0f })
    {
        math::Vector<float, 1> q{ angle };
        auto positions = fk1.Compute(links, q);

        EXPECT_NEAR(positions[0].at(0, 0), 0.0f, 1e-6f);
        EXPECT_NEAR(positions[0].at(1, 0), 0.0f, 1e-6f);
        EXPECT_NEAR(positions[0].at(2, 0), 0.0f, 1e-6f);
    }
}

TEST_F(TestForwardKinematics, end_effector_distance_equals_link_length)
{
    float length = 1.0f;
    auto link = MakePlanarLink(1.0f, length, math::Vector<float, 3>{});
    std::array<dynamics::RevoluteJointLink<float>, 1> links = { link };

    for (float angle : { 0.0f, pi / 6.0f, pi / 3.0f, pi / 2.0f, 2.0f * pi / 3.0f })
    {
        math::Vector<float, 1> q{ angle };
        auto positions = fk1.Compute(links, q);

        float dx = positions[1].at(0, 0) - positions[0].at(0, 0);
        float dy = positions[1].at(1, 0) - positions[0].at(1, 0);
        float dz = positions[1].at(2, 0) - positions[0].at(2, 0);
        float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        EXPECT_NEAR(dist, length, tolerance);
    }
}

TEST_F(TestForwardKinematics, three_link_all_zero)
{
    float l1 = 1.0f, l2 = 0.8f, l3 = 0.6f;

    auto link1 = MakePlanarLink(1.0f, l1, math::Vector<float, 3>{});
    auto link2 = MakePlanarLink(0.8f, l2, math::Vector<float, 3>{ l1, 0.0f, 0.0f });
    auto link3 = MakePlanarLink(0.6f, l3, math::Vector<float, 3>{ l2, 0.0f, 0.0f });
    std::array<dynamics::RevoluteJointLink<float>, 3> links = { link1, link2, link3 };

    math::Vector<float, 3> q{ 0.0f, 0.0f, 0.0f };
    auto positions = fk3.Compute(links, q);

    EXPECT_NEAR(positions[0].at(0, 0), 0.0f, tolerance);
    EXPECT_NEAR(positions[1].at(0, 0), l1, tolerance);
    EXPECT_NEAR(positions[2].at(0, 0), l1 + l2, tolerance);
    EXPECT_NEAR(positions[3].at(0, 0), l1 + l2 + l3, tolerance);

    for (int i = 0; i <= 3; ++i)
    {
        EXPECT_NEAR(positions[i].at(1, 0), 0.0f, tolerance);
        EXPECT_NEAR(positions[i].at(2, 0), 0.0f, tolerance);
    }
}

TEST_F(TestForwardKinematics, two_link_total_reach)
{
    float l1 = 1.0f, l2 = 0.5f;

    auto link1 = MakePlanarLink(1.0f, l1, math::Vector<float, 3>{});
    auto link2 = MakePlanarLink(1.0f, l2, math::Vector<float, 3>{ l1, 0.0f, 0.0f });
    std::array<dynamics::RevoluteJointLink<float>, 2> links = { link1, link2 };

    for (float q1 : { 0.0f, pi / 4.0f, pi / 2.0f })
    {
        for (float q2 : { 0.0f, pi / 3.0f, pi, -pi / 2.0f })
        {
            math::Vector<float, 2> q{ q1, q2 };
            auto positions = fk2.Compute(links, q);

            float x = positions[2].at(0, 0);
            float y = positions[2].at(1, 0);
            float z = positions[2].at(2, 0);
            float dist = std::sqrt(x * x + y * y + z * z);

            EXPECT_LE(dist, l1 + l2 + 1e-3f);
        }
    }
}

TEST_F(TestForwardKinematics, spatial_3dof_vertical_base)
{
    float l1 = 0.5f, l2 = 0.4f, l3 = 0.3f;

    auto link1 = MakeLink(1.0f, l1,
        math::Vector<float, 3>{ 0.0f, 0.0f, 1.0f },
        math::Vector<float, 3>{},
        math::Vector<float, 3>{ 0.0f, 0.0f, l1 / 2.0f });

    auto link2 = MakeLink(0.8f, l2,
        math::Vector<float, 3>{ 0.0f, 1.0f, 0.0f },
        math::Vector<float, 3>{ 0.0f, 0.0f, l1 },
        math::Vector<float, 3>{ l2 / 2.0f, 0.0f, 0.0f });

    auto link3 = MakeLink(0.6f, l3,
        math::Vector<float, 3>{ 0.0f, 1.0f, 0.0f },
        math::Vector<float, 3>{ l2, 0.0f, 0.0f },
        math::Vector<float, 3>{ l3 / 2.0f, 0.0f, 0.0f });

    std::array<dynamics::RevoluteJointLink<float>, 3> links = { link1, link2, link3 };

    math::Vector<float, 3> q{ 0.0f, 0.0f, 0.0f };
    auto positions = fk3.Compute(links, q);

    EXPECT_NEAR(positions[1].at(2, 0), l1, tolerance);
    EXPECT_NEAR(positions[1].at(0, 0), 0.0f, tolerance);

    EXPECT_NEAR(positions[2].at(0, 0), l2, tolerance);
    EXPECT_NEAR(positions[2].at(2, 0), l1, tolerance);

    EXPECT_NEAR(positions[3].at(0, 0), l2 + l3, tolerance);
    EXPECT_NEAR(positions[3].at(2, 0), l1, tolerance);
}
