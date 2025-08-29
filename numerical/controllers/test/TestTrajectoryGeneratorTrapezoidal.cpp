#include "numerical/controllers/TrajectoryGeneratorTrapezoidal.hpp"
#include <gtest/gtest.h>
#include <vector>

namespace
{
    template<typename T>
    class TestTrajectoryGenerator
        : public ::testing::Test
    {
    public:
        TestTrajectoryGenerator()
            : constraints{ T(10.0f), T(5.0f), T(5.0f) }
            , generator(constraints)
        {
        }

        controllers::TrajectoryGeneratorTrapezoidal<T>::Constraints constraints;
        controllers::TrajectoryGeneratorTrapezoidal<T> generator;

        void RunTrajectory(T targetPosition, T timeStep = T(0.01))
        {
            generator.SetTarget(targetPosition);

            trajectory.clear();
            T time = T(0);

            while (!generator.IsComplete() && time < T(10.0))
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
    this->generator.SetInitialConditions(TypeParam(5.0f), TypeParam(0.0f));
    this->RunTrajectory(TypeParam(5.0));

    EXPECT_TRUE(this->generator.IsComplete());
    EXPECT_TRUE(this->trajectory.empty() || this->trajectory.size() == 1);
}

TYPED_TEST(TestTrajectoryGenerator, TriangularProfile)
{
    this->generator.SetInitialConditions(TypeParam(0.0f), TypeParam(0.0f));
    this->RunTrajectory(TypeParam(5.0));

    EXPECT_FALSE(this->trajectory.empty());
    EXPECT_TRUE(this->generator.IsComplete());

    auto finalPoint = this->trajectory.back();
    EXPECT_NEAR(finalPoint.position, 5.0, 0.1);
    EXPECT_NEAR(finalPoint.velocity, 0.0, 0.1);
}

TYPED_TEST(TestTrajectoryGenerator, TrapezoidalProfile)
{
    this->generator.SetInitialConditions(TypeParam(0.0f), TypeParam(0.0f));
    this->RunTrajectory(TypeParam(50.0f));

    EXPECT_FALSE(this->trajectory.empty());
    EXPECT_TRUE(this->generator.IsComplete());

    auto finalPoint = this->trajectory.back();
    EXPECT_NEAR(finalPoint.position, 50.0f, 0.1f);
    EXPECT_NEAR(finalPoint.velocity, 0.0f, 0.1f);

    TypeParam maxVel = TypeParam(0);
    for (const auto& point : this->trajectory)
        maxVel = std::max(maxVel, std::abs(point.velocity));

    EXPECT_LE(maxVel, this->constraints.maxVelocity + TypeParam(0.1f));
}

TYPED_TEST(TestTrajectoryGenerator, AccelerationConstraints)
{
    this->generator.SetInitialConditions(TypeParam(0.0f), TypeParam(0.0f));
    this->RunTrajectory(TypeParam(20.0));

    for (const auto& point : this->trajectory)
        EXPECT_LE(std::abs(point.acceleration), this->constraints.maxAcceleration + TypeParam(0.1f));
}

TYPED_TEST(TestTrajectoryGenerator, NegativeDirection)
{
    this->generator.SetInitialConditions(TypeParam(10.0f), TypeParam(0.0f));
    this->RunTrajectory(TypeParam(0.0f));

    EXPECT_FALSE(this->trajectory.empty());
    EXPECT_TRUE(this->generator.IsComplete());

    auto finalPoint = this->trajectory.back();
    EXPECT_NEAR(finalPoint.position, 0.0f, 0.1f);
    EXPECT_NEAR(finalPoint.velocity, 0.0f, 0.1f);
}

TYPED_TEST(TestTrajectoryGenerator, MultipleTargets)
{
    this->generator.SetInitialConditions(TypeParam(0.0f), TypeParam(0.0f));

    this->RunTrajectory(TypeParam(10.0f));
    EXPECT_TRUE(this->generator.IsComplete());

    TypeParam currentPos = this->trajectory.empty() ? TypeParam(0) : this->trajectory.back().position;
    this->generator.SetInitialConditions(currentPos, TypeParam(0.0f));
    this->RunTrajectory(TypeParam(5.0f));
    EXPECT_TRUE(this->generator.IsComplete());
}

TYPED_TEST(TestTrajectoryGenerator, ResetFunctionality)
{
    this->generator.SetInitialConditions(TypeParam(0.0f), TypeParam(0.0f));
    this->generator.SetTarget(TypeParam(10.0f));

    this->generator.Update(TypeParam(0.1f));
    this->generator.Update(TypeParam(0.1f));

    this->generator.Reset(TypeParam(5.0f));
    EXPECT_TRUE(this->generator.IsComplete());

    auto profile = this->generator.Update(TypeParam(0.01f));
    EXPECT_NEAR(profile.position, 5.0f, 0.01f);
    EXPECT_NEAR(profile.velocity, 0.0f, 0.01f);
}

TYPED_TEST(TestTrajectoryGenerator, SmoothnessTest)
{
    this->generator.SetInitialConditions(TypeParam(0.0f), TypeParam(0.0f));
    this->RunTrajectory(TypeParam(20.0f), TypeParam(0.001f));

    for (size_t i = 1; i < this->trajectory.size(); ++i)
    {
        TypeParam dt = this->trajectory[i].time - this->trajectory[i - 1].time;
        TypeParam positionDiff = this->trajectory[i].position - this->trajectory[i - 1].position;
        TypeParam expectedPositionDiff = this->trajectory[i - 1].velocity * dt;

        EXPECT_NEAR(positionDiff, expectedPositionDiff, 0.001);
    }
}

TYPED_TEST(TestTrajectoryGenerator, VelocityProfile)
{
    this->generator.SetInitialConditions(TypeParam(0.0f), TypeParam(0.0f));
    this->RunTrajectory(TypeParam(30.0f));

    bool foundMaxVelocity = false;
    for (const auto& point : this->trajectory)
        if (std::abs(point.velocity - this->constraints.maxVelocity) < TypeParam(0.1f))
        {
            foundMaxVelocity = true;
            break;
        }

    EXPECT_TRUE(foundMaxVelocity) << "Max velocity should be reached in trapezoidal profile";
}
