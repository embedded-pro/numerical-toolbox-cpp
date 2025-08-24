#include "numerical/controllers/TrajectoryGenerator.hpp"
#include <gtest/gtest.h>
#include <vector>

namespace
{
    template<typename T>
    class TestTrajectoryGenerator : public ::testing::Test
    {
    public:
        TestTrajectoryGenerator()
            : constraints{ T(10.0), T(5.0), T(5.0) } // maxVel=10, maxAcc=5, maxDec=5
            , generator(constraints)
        {
        }

        controllers::TrajectoryGenerator<T>::Constraints constraints;
        controllers::TrajectoryGenerator<T> generator;

        void RunTrajectory(T targetPosition, T timeStep = T(0.01))
        {
            generator.SetTarget(targetPosition);

            trajectory.clear();
            T time = T(0);

            while (!generator.IsComplete() && time < T(10.0)) // Safety limit
            {
                auto profile = generator.Update(timeStep);
                trajectory.push_back({ time, profile.position, profile.velocity, profile.acceleration });
                time += timeStep;
            }
        }

        struct TrajectoryPoint
        {
            T time;
            T position;
            T velocity;
            T acceleration;
        };

        std::vector<TrajectoryPoint> trajectory;
    };

    using TestTypes = ::testing::Types<float, double>;
    TYPED_TEST_SUITE(TestTrajectoryGenerator, TestTypes);
}

TYPED_TEST(TestTrajectoryGenerator, ZeroDistance)
{
    // Test case where target equals current position
    this->generator.SetInitialConditions(TypeParam(5.0));
    this->RunTrajectory(TypeParam(5.0));

    EXPECT_TRUE(this->generator.IsComplete());
    EXPECT_TRUE(this->trajectory.empty() || this->trajectory.size() == 1);
}

TYPED_TEST(TestTrajectoryGenerator, TriangularProfile)
{
    // Short distance requiring triangular profile
    this->generator.SetInitialConditions(TypeParam(0.0));
    this->RunTrajectory(TypeParam(5.0));

    EXPECT_FALSE(this->trajectory.empty());
    EXPECT_TRUE(this->generator.IsComplete());

    // Check final position
    auto finalPoint = this->trajectory.back();
    EXPECT_NEAR(finalPoint.position, 5.0, 0.1);
    EXPECT_NEAR(finalPoint.velocity, 0.0, 0.1);
}

TYPED_TEST(TestTrajectoryGenerator, TrapezoidalProfile)
{
    // Long distance requiring trapezoidal profile
    this->generator.SetInitialConditions(TypeParam(0.0));
    this->RunTrajectory(TypeParam(50.0));

    EXPECT_FALSE(this->trajectory.empty());
    EXPECT_TRUE(this->generator.IsComplete());

    // Check final position
    auto finalPoint = this->trajectory.back();
    EXPECT_NEAR(finalPoint.position, 50.0, 0.1);
    EXPECT_NEAR(finalPoint.velocity, 0.0, 0.1);

    // Check that max velocity constraint is respected
    TypeParam maxVel = TypeParam(0);
    for (const auto& point : this->trajectory)
    {
        maxVel = std::max(maxVel, std::abs(point.velocity));
    }
    EXPECT_LE(maxVel, this->constraints.maxVelocity + TypeParam(0.1));
}

TYPED_TEST(TestTrajectoryGenerator, AccelerationConstraints)
{
    this->generator.SetInitialConditions(TypeParam(0.0));
    this->RunTrajectory(TypeParam(20.0));

    // Check that acceleration constraints are respected
    for (const auto& point : this->trajectory)
    {
        EXPECT_LE(std::abs(point.acceleration), this->constraints.maxAcceleration + TypeParam(0.1));
    }
}

TYPED_TEST(TestTrajectoryGenerator, NegativeDirection)
{
    this->generator.SetInitialConditions(TypeParam(10.0));
    this->RunTrajectory(TypeParam(0.0));

    EXPECT_FALSE(this->trajectory.empty());
    EXPECT_TRUE(this->generator.IsComplete());

    auto finalPoint = this->trajectory.back();
    EXPECT_NEAR(finalPoint.position, 0.0, 0.1);
    EXPECT_NEAR(finalPoint.velocity, 0.0, 0.1);
}

TYPED_TEST(TestTrajectoryGenerator, MultipleTargets)
{
    // Test sequential trajectory generation
    this->generator.SetInitialConditions(TypeParam(0.0));

    // First trajectory
    this->RunTrajectory(TypeParam(10.0));
    EXPECT_TRUE(this->generator.IsComplete());

    // Second trajectory from current position
    TypeParam currentPos = this->trajectory.empty() ? TypeParam(0) : this->trajectory.back().position;
    this->generator.SetInitialConditions(currentPos);
    this->RunTrajectory(TypeParam(5.0));
    EXPECT_TRUE(this->generator.IsComplete());
}

TYPED_TEST(TestTrajectoryGenerator, ResetFunctionality)
{
    this->generator.SetInitialConditions(TypeParam(0.0));
    this->generator.SetTarget(TypeParam(10.0));

    // Update a few times
    this->generator.Update(TypeParam(0.1));
    this->generator.Update(TypeParam(0.1));

    // Reset and check
    this->generator.Reset(TypeParam(5.0));
    EXPECT_TRUE(this->generator.IsComplete());

    auto profile = this->generator.Update(TypeParam(0.01));
    EXPECT_NEAR(profile.position, 5.0, 0.01);
    EXPECT_NEAR(profile.velocity, 0.0, 0.01);
}

TYPED_TEST(TestTrajectoryGenerator, SmoothnessTest)
{
    this->generator.SetInitialConditions(TypeParam(0.0));
    this->RunTrajectory(TypeParam(20.0), TypeParam(0.001)); // Small time step

    // Check for continuity in position and velocity
    for (size_t i = 1; i < this->trajectory.size(); ++i)
    {
        TypeParam dt = this->trajectory[i].time - this->trajectory[i - 1].time;
        TypeParam positionDiff = this->trajectory[i].position - this->trajectory[i - 1].position;
        TypeParam expectedPositionDiff = this->trajectory[i - 1].velocity * dt;

        // Position should be continuous
        EXPECT_NEAR(positionDiff, expectedPositionDiff, 0.001);
    }
}

TYPED_TEST(TestTrajectoryGenerator, VelocityProfile)
{
    this->generator.SetInitialConditions(TypeParam(0.0));
    this->RunTrajectory(TypeParam(30.0)); // Should create trapezoidal profile

    bool foundMaxVelocity = false;
    for (const auto& point : this->trajectory)
    {
        if (std::abs(point.velocity - this->constraints.maxVelocity) < TypeParam(0.1))
        {
            foundMaxVelocity = true;
            break;
        }
    }

    EXPECT_TRUE(foundMaxVelocity) << "Max velocity should be reached in trapezoidal profile";
}
