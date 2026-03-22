#include "numerical/dynamics/NewtonEulerBody.hpp"
#include "numerical/dynamics/NewtonEulerSolver.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    // Uniform sphere: I = (2/5) * m * r^2 * Identity
    class UniformSphere : public dynamics::NewtonEulerBody<float>
    {
    public:
        static constexpr float mass = 2.0f;
        static constexpr float radius = 0.5f;

        float ComputeMass() const override
        {
            return mass;
        }

        InertiaMatrix ComputeInertia() const override
        {
            float i = 0.4f * mass * radius * radius; // (2/5) * m * r^2
            return InertiaMatrix{
                { i, 0.0f, 0.0f },
                { 0.0f, i, 0.0f },
                { 0.0f, 0.0f, i }
            };
        }
    };

    // Asymmetric body with distinct principal moments of inertia
    class AsymmetricBody : public dynamics::NewtonEulerBody<float>
    {
    public:
        static constexpr float mass = 3.0f;

        float ComputeMass() const override
        {
            return mass;
        }

        InertiaMatrix ComputeInertia() const override
        {
            return InertiaMatrix{
                { 1.0f, 0.0f, 0.0f },
                { 0.0f, 2.0f, 0.0f },
                { 0.0f, 0.0f, 3.0f }
            };
        }
    };

    class TestNewtonEulerSolver : public ::testing::Test
    {
    protected:
        dynamics::NewtonEulerSolver<float> solver;
        UniformSphere sphere;
        AsymmetricBody asymmetricBody;

        math::Vector<float, 3> zero3{};
    };
}

TEST_F(TestNewtonEulerSolver, forward_dynamics_pure_translation_no_rotation)
{
    math::Vector<float, 3> force{ 6.0f, 0.0f, 0.0f };
    math::Vector<float, 3> torque{};

    auto result = solver.ForwardDynamics(sphere, force, torque, zero3, zero3);

    // a = F / m = 6 / 2 = 3
    EXPECT_NEAR(result.linear.at(0, 0), 3.0f, 1e-4f);
    EXPECT_NEAR(result.linear.at(1, 0), 0.0f, 1e-4f);
    EXPECT_NEAR(result.linear.at(2, 0), 0.0f, 1e-4f);

    EXPECT_NEAR(result.angular.at(0, 0), 0.0f, 1e-4f);
    EXPECT_NEAR(result.angular.at(1, 0), 0.0f, 1e-4f);
    EXPECT_NEAR(result.angular.at(2, 0), 0.0f, 1e-4f);
}

TEST_F(TestNewtonEulerSolver, forward_dynamics_pure_rotation_no_translation)
{
    math::Vector<float, 3> force{};
    math::Vector<float, 3> torque{ 0.0f, 0.0f, 1.0f };

    auto result = solver.ForwardDynamics(sphere, force, torque, zero3, zero3);

    // alpha = I^{-1} * torque, I = 0.2 * Identity → alpha_z = 1.0 / 0.2 = 5.0
    float inertia = 0.4f * UniformSphere::mass * UniformSphere::radius * UniformSphere::radius;
    EXPECT_NEAR(result.angular.at(2, 0), 1.0f / inertia, 1e-3f);
    EXPECT_NEAR(result.linear.at(0, 0), 0.0f, 1e-4f);
    EXPECT_NEAR(result.linear.at(1, 0), 0.0f, 1e-4f);
    EXPECT_NEAR(result.linear.at(2, 0), 0.0f, 1e-4f);
}

TEST_F(TestNewtonEulerSolver, inverse_dynamics_at_rest_returns_zero)
{
    auto result = solver.InverseDynamics(sphere, zero3, zero3, zero3, zero3);

    EXPECT_NEAR(result.force.at(0, 0), 0.0f, 1e-6f);
    EXPECT_NEAR(result.force.at(1, 0), 0.0f, 1e-6f);
    EXPECT_NEAR(result.force.at(2, 0), 0.0f, 1e-6f);
    EXPECT_NEAR(result.torque.at(0, 0), 0.0f, 1e-6f);
    EXPECT_NEAR(result.torque.at(1, 0), 0.0f, 1e-6f);
    EXPECT_NEAR(result.torque.at(2, 0), 0.0f, 1e-6f);
}

TEST_F(TestNewtonEulerSolver, forward_inverse_roundtrip_consistency)
{
    math::Vector<float, 3> force{ 1.0f, -2.0f, 3.0f };
    math::Vector<float, 3> torque{ 0.5f, -0.3f, 0.8f };
    math::Vector<float, 3> linearVel{ 0.1f, 0.2f, -0.1f };
    math::Vector<float, 3> angularVel{ 0.3f, -0.5f, 0.2f };

    auto accel = solver.ForwardDynamics(sphere, force, torque, linearVel, angularVel);
    auto recovered = solver.InverseDynamics(sphere, accel.linear, accel.angular, linearVel, angularVel);

    EXPECT_NEAR(recovered.force.at(0, 0), force.at(0, 0), 1e-3f);
    EXPECT_NEAR(recovered.force.at(1, 0), force.at(1, 0), 1e-3f);
    EXPECT_NEAR(recovered.force.at(2, 0), force.at(2, 0), 1e-3f);
    EXPECT_NEAR(recovered.torque.at(0, 0), torque.at(0, 0), 1e-3f);
    EXPECT_NEAR(recovered.torque.at(1, 0), torque.at(1, 0), 1e-3f);
    EXPECT_NEAR(recovered.torque.at(2, 0), torque.at(2, 0), 1e-3f);
}

TEST_F(TestNewtonEulerSolver, gyroscopic_effect_on_asymmetric_body)
{
    // Spinning about z-axis with no external torque → gyroscopic coupling
    math::Vector<float, 3> force{};
    math::Vector<float, 3> torque{};
    math::Vector<float, 3> angularVel{ 0.0f, 0.0f, 1.0f };

    auto result = solver.ForwardDynamics(asymmetricBody, force, torque, zero3, angularVel);

    // omega x (I * omega) = [0,0,1] x [0,0,3] = [0*3 - 1*0, 1*0 - 0*3, 0*0 - 0*0] = [0,0,0]
    // When omega is aligned with a principal axis, gyroscopic term vanishes
    EXPECT_NEAR(result.angular.at(0, 0), 0.0f, 1e-4f);
    EXPECT_NEAR(result.angular.at(1, 0), 0.0f, 1e-4f);
    EXPECT_NEAR(result.angular.at(2, 0), 0.0f, 1e-4f);
}

TEST_F(TestNewtonEulerSolver, gyroscopic_effect_off_principal_axis)
{
    // Spinning about combined axes on asymmetric body → non-zero gyroscopic term
    math::Vector<float, 3> force{};
    math::Vector<float, 3> torque{};
    math::Vector<float, 3> angularVel{ 1.0f, 1.0f, 0.0f };

    auto result = solver.ForwardDynamics(asymmetricBody, force, torque, zero3, angularVel);

    // I*omega = [1*1, 2*1, 0] = [1, 2, 0]
    // omega x I*omega = [1,1,0] x [1,2,0] = [0-0, 0-0, 2-1] = [0, 0, 1]
    // alpha = I^{-1} * (0 - [0,0,1]) = [0, 0, -1/3]
    EXPECT_NEAR(result.angular.at(0, 0), 0.0f, 1e-4f);
    EXPECT_NEAR(result.angular.at(1, 0), 0.0f, 1e-4f);
    EXPECT_NEAR(result.angular.at(2, 0), -1.0f / 3.0f, 1e-3f);
}

TEST_F(TestNewtonEulerSolver, body_frame_coriolis_effect)
{
    // Linear velocity + angular velocity → omega x v contributes to forward dynamics
    math::Vector<float, 3> force{};
    math::Vector<float, 3> torque{};
    math::Vector<float, 3> linearVel{ 1.0f, 0.0f, 0.0f };
    math::Vector<float, 3> angularVel{ 0.0f, 0.0f, 1.0f };

    auto result = solver.ForwardDynamics(sphere, force, torque, linearVel, angularVel);

    // a = F/m - omega x v = 0 - [0,0,1] x [1,0,0] = -[0,0,0 x 1,0,0] = -[0*0-1*0, 1*1-0*0, 0*0-0*1] = -[0, 1, 0]
    EXPECT_NEAR(result.linear.at(0, 0), 0.0f, 1e-4f);
    EXPECT_NEAR(result.linear.at(1, 0), -1.0f, 1e-4f);
    EXPECT_NEAR(result.linear.at(2, 0), 0.0f, 1e-4f);
}

TEST_F(TestNewtonEulerSolver, asymmetric_body_forward_inverse_roundtrip)
{
    math::Vector<float, 3> force{ 3.0f, -1.0f, 2.0f };
    math::Vector<float, 3> torque{ 1.0f, 0.5f, -0.7f };
    math::Vector<float, 3> linearVel{ 0.5f, -0.3f, 0.8f };
    math::Vector<float, 3> angularVel{ 0.2f, 0.4f, -0.6f };

    auto accel = solver.ForwardDynamics(asymmetricBody, force, torque, linearVel, angularVel);
    auto recovered = solver.InverseDynamics(asymmetricBody, accel.linear, accel.angular, linearVel, angularVel);

    EXPECT_NEAR(recovered.force.at(0, 0), force.at(0, 0), 1e-3f);
    EXPECT_NEAR(recovered.force.at(1, 0), force.at(1, 0), 1e-3f);
    EXPECT_NEAR(recovered.force.at(2, 0), force.at(2, 0), 1e-3f);
    EXPECT_NEAR(recovered.torque.at(0, 0), torque.at(0, 0), 1e-3f);
    EXPECT_NEAR(recovered.torque.at(1, 0), torque.at(1, 0), 1e-3f);
    EXPECT_NEAR(recovered.torque.at(2, 0), torque.at(2, 0), 1e-3f);
}
