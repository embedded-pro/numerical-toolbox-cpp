#include "numerical/kinematics/ForwardKinematics.hpp"
#include "numerical/kinematics/InverseKinematics.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    constexpr float tolerance = 1e-3f;
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

    class TestInverseKinematics : public ::testing::Test
    {
    protected:
        kinematics::InverseKinematics<float, 1> ik1;
        kinematics::InverseKinematics<float, 2> ik2;
        kinematics::InverseKinematics<float, 3> ik3;
        kinematics::ForwardKinematics<float, 1> fk1;
        kinematics::ForwardKinematics<float, 2> fk2;
        kinematics::ForwardKinematics<float, 3> fk3;
    };
}

TEST_F(TestInverseKinematics, single_link_reaches_target_on_arc)
{
    float length = 1.0f;
    auto link = MakePlanarLink(1.0f, length, math::Vector<float, 3>{});
    std::array<dynamics::RevoluteJointLink<float>, 1> links = { link };

    math::Vector<float, 1> referenceQ{ pi / 3.0f };
    auto fkRef = fk1.Compute(links, referenceQ);
    math::Vector<float, 3> target = fkRef[1];

    math::Vector<float, 1> initialQ{ 0.0f };
    auto result = ik1.Solve(links, target, initialQ);

    EXPECT_TRUE(result.converged);
    EXPECT_NEAR(result.q.at(0, 0), pi / 3.0f, 1e-3f);

    auto positions = fk1.Compute(links, result.q);
    EXPECT_NEAR(positions[1].at(0, 0), target.at(0, 0), tolerance);
    EXPECT_NEAR(positions[1].at(1, 0), target.at(1, 0), tolerance);
    EXPECT_NEAR(positions[1].at(2, 0), target.at(2, 0), tolerance);
}

TEST_F(TestInverseKinematics, starts_at_solution_converges_immediately)
{
    float length = 1.0f;
    auto link = MakePlanarLink(1.0f, length, math::Vector<float, 3>{});
    std::array<dynamics::RevoluteJointLink<float>, 1> links = { link };

    math::Vector<float, 1> initialQ{ pi / 4.0f };
    auto fkPositions = fk1.Compute(links, initialQ);
    math::Vector<float, 3> target = fkPositions[1];

    auto result = ik1.Solve(links, target, initialQ);

    EXPECT_TRUE(result.converged);
    EXPECT_LT(result.iterations, 2u);
    EXPECT_LT(result.finalError, 1e-4f);
}

TEST_F(TestInverseKinematics, two_link_planar_reaches_known_position)
{
    float l1 = 1.0f, l2 = 0.8f;

    auto link1 = MakePlanarLink(1.0f, l1, math::Vector<float, 3>{});
    auto link2 = MakePlanarLink(0.8f, l2, math::Vector<float, 3>{ l1, 0.0f, 0.0f });
    std::array<dynamics::RevoluteJointLink<float>, 2> links = { link1, link2 };

    math::Vector<float, 2> initialQ{ 0.0f, 0.0f };
    math::Vector<float, 3> target{ 0.5f, 1.0f, 0.0f };

    auto result = ik2.Solve(links, target, initialQ);

    EXPECT_TRUE(result.converged);

    auto positions = fk2.Compute(links, result.q);
    EXPECT_NEAR(positions[2].at(0, 0), target.at(0, 0), tolerance);
    EXPECT_NEAR(positions[2].at(1, 0), target.at(1, 0), tolerance);
    EXPECT_NEAR(positions[2].at(2, 0), target.at(2, 0), tolerance);
}

TEST_F(TestInverseKinematics, two_link_planar_fully_extended)
{
    float l1 = 1.0f, l2 = 0.5f;

    auto link1 = MakePlanarLink(1.0f, l1, math::Vector<float, 3>{});
    auto link2 = MakePlanarLink(0.5f, l2, math::Vector<float, 3>{ l1, 0.0f, 0.0f });
    std::array<dynamics::RevoluteJointLink<float>, 2> links = { link1, link2 };

    math::Vector<float, 2> initialQ{ 0.1f, 0.1f };
    math::Vector<float, 3> target{ l1 + l2 - 0.05f, 0.0f, 0.0f };

    auto result = ik2.Solve(links, target, initialQ);

    EXPECT_TRUE(result.converged);

    auto positions = fk2.Compute(links, result.q);
    EXPECT_NEAR(positions[2].at(0, 0), target.at(0, 0), tolerance);
    EXPECT_NEAR(positions[2].at(1, 0), 0.0f, tolerance);
}

TEST_F(TestInverseKinematics, two_link_planar_folded_elbow_up)
{
    float l1 = 1.0f, l2 = 0.8f;

    auto link1 = MakePlanarLink(1.0f, l1, math::Vector<float, 3>{});
    auto link2 = MakePlanarLink(0.8f, l2, math::Vector<float, 3>{ l1, 0.0f, 0.0f });
    std::array<dynamics::RevoluteJointLink<float>, 2> links = { link1, link2 };

    math::Vector<float, 2> initialQ{ pi / 4.0f, -pi / 4.0f };
    math::Vector<float, 3> target{ 1.2f, 0.5f, 0.0f };

    auto result = ik2.Solve(links, target, initialQ);

    EXPECT_TRUE(result.converged);

    auto positions = fk2.Compute(links, result.q);
    EXPECT_NEAR(positions[2].at(0, 0), target.at(0, 0), tolerance);
    EXPECT_NEAR(positions[2].at(1, 0), target.at(1, 0), tolerance);
}

TEST_F(TestInverseKinematics, unreachable_target_does_not_converge)
{
    float l1 = 1.0f, l2 = 0.5f;

    auto link1 = MakePlanarLink(1.0f, l1, math::Vector<float, 3>{});
    auto link2 = MakePlanarLink(0.5f, l2, math::Vector<float, 3>{ l1, 0.0f, 0.0f });
    std::array<dynamics::RevoluteJointLink<float>, 2> links = { link1, link2 };

    math::Vector<float, 2> initialQ{ 0.0f, 0.0f };
    math::Vector<float, 3> target{ 10.0f, 10.0f, 10.0f };

    auto result = ik2.Solve(links, target, initialQ);

    EXPECT_FALSE(result.converged);
    EXPECT_GT(result.finalError, 1e-4f);
    EXPECT_EQ(result.iterations, 100u);
}

TEST_F(TestInverseKinematics, result_contains_populated_iteration_count_and_error)
{
    float l1 = 1.0f, l2 = 0.8f;

    auto link1 = MakePlanarLink(1.0f, l1, math::Vector<float, 3>{});
    auto link2 = MakePlanarLink(0.8f, l2, math::Vector<float, 3>{ l1, 0.0f, 0.0f });
    std::array<dynamics::RevoluteJointLink<float>, 2> links = { link1, link2 };

    math::Vector<float, 2> initialQ{ 0.0f, 0.0f };
    math::Vector<float, 3> target{ 0.5f, 1.0f, 0.0f };

    auto result = ik2.Solve(links, target, initialQ);

    EXPECT_GT(result.iterations, 0u);
    EXPECT_LT(result.finalError, 1e-4f);
}

TEST_F(TestInverseKinematics, custom_damping_factor_still_converges)
{
    float l1 = 1.0f, l2 = 0.8f;

    auto link1 = MakePlanarLink(1.0f, l1, math::Vector<float, 3>{});
    auto link2 = MakePlanarLink(0.8f, l2, math::Vector<float, 3>{ l1, 0.0f, 0.0f });
    std::array<dynamics::RevoluteJointLink<float>, 2> links = { link1, link2 };

    kinematics::InverseKinematicsConfig<float> cfg;
    cfg.dampingFactor = 0.5f;
    cfg.tolerance = 1e-4f;
    cfg.maxIterations = 500;
    kinematics::InverseKinematics<float, 2> ikHighDamping{ cfg };

    math::Vector<float, 2> initialQ{ 0.0f, 0.0f };
    math::Vector<float, 3> target{ 0.5f, 1.0f, 0.0f };

    auto result = ikHighDamping.Solve(links, target, initialQ);

    EXPECT_TRUE(result.converged);

    auto positions = fk2.Compute(links, result.q);
    EXPECT_NEAR(positions[2].at(0, 0), target.at(0, 0), tolerance);
    EXPECT_NEAR(positions[2].at(1, 0), target.at(1, 0), tolerance);
}

TEST_F(TestInverseKinematics, three_link_spatial_reaches_target)
{
    float l1 = 0.5f, l2 = 0.4f, l3 = 0.3f;

    auto link1 = MakeLink(1.0f, l1,
        math::Vector<float, 3>{ 0.0f, 0.0f, 1.0f },
        math::Vector<float, 3>{},
        math::Vector<float, 3>{ l1 / 2.0f, 0.0f, 0.0f });
    auto link2 = MakeLink(0.8f, l2,
        math::Vector<float, 3>{ 0.0f, 1.0f, 0.0f },
        math::Vector<float, 3>{ l1, 0.0f, 0.0f },
        math::Vector<float, 3>{ l2 / 2.0f, 0.0f, 0.0f });
    auto link3 = MakeLink(0.6f, l3,
        math::Vector<float, 3>{ 0.0f, 1.0f, 0.0f },
        math::Vector<float, 3>{ l2, 0.0f, 0.0f },
        math::Vector<float, 3>{ l3 / 2.0f, 0.0f, 0.0f });
    std::array<dynamics::RevoluteJointLink<float>, 3> links = { link1, link2, link3 };

    kinematics::InverseKinematicsConfig<float> cfg;
    cfg.dampingFactor = 0.05f;
    cfg.tolerance = 1e-4f;
    cfg.maxIterations = 300;
    kinematics::InverseKinematics<float, 3> ik3Spatial{ cfg };

    math::Vector<float, 3> referenceQ{ pi / 4.0f, pi / 6.0f, -pi / 3.0f };
    auto fkRef = fk3.Compute(links, referenceQ);
    math::Vector<float, 3> target = fkRef[3];

    math::Vector<float, 3> initialQ{ 0.0f, 0.0f, 0.0f };
    auto result = ik3Spatial.Solve(links, target, initialQ);

    EXPECT_TRUE(result.converged);

    auto positions = fk3.Compute(links, result.q);
    EXPECT_NEAR(positions[3].at(0, 0), target.at(0, 0), tolerance);
    EXPECT_NEAR(positions[3].at(1, 0), target.at(1, 0), tolerance);
    EXPECT_NEAR(positions[3].at(2, 0), target.at(2, 0), tolerance);
}

TEST_F(TestInverseKinematics, two_link_target_at_origin_region)
{
    float l1 = 1.0f, l2 = 0.8f;

    auto link1 = MakePlanarLink(1.0f, l1, math::Vector<float, 3>{});
    auto link2 = MakePlanarLink(0.8f, l2, math::Vector<float, 3>{ l1, 0.0f, 0.0f });
    std::array<dynamics::RevoluteJointLink<float>, 2> links = { link1, link2 };

    math::Vector<float, 2> initialQ{ pi / 3.0f, -2.0f * pi / 3.0f };
    math::Vector<float, 3> target{ 0.4f, 0.0f, 0.0f };

    auto result = ik2.Solve(links, target, initialQ);

    EXPECT_TRUE(result.converged);

    auto positions = fk2.Compute(links, result.q);
    EXPECT_NEAR(positions[2].at(0, 0), target.at(0, 0), tolerance);
    EXPECT_NEAR(positions[2].at(1, 0), target.at(1, 0), tolerance);
}
