#include "numerical/math/SE3Transform.hpp"
#include <cmath>
#include <cstddef>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    using Transform = math::SE3Transform<float>;
    using Vector3 = math::Vector3<float>;
    using Vector6 = math::Vector6<float>;

    class TestSE3Transform : public ::testing::Test
    {
    protected:
        static constexpr float pi{ std::numbers::pi_v<float> };
        static constexpr float tolerance{ 1e-5f };
        const Vector3 xAxis{ 1.0f, 0.0f, 0.0f };
        const Vector3 yAxis{ 0.0f, 1.0f, 0.0f };
        const Vector3 zAxis{ 0.0f, 0.0f, 1.0f };
        const Transform a{ math::RotationAboutAxis(zAxis, pi / 2.0f), Vector3{ 1.0f, 0.0f, 0.0f } };
        const Transform b{ math::RotationAboutAxis(yAxis, pi / 3.0f), Vector3{ 0.0f, 2.0f, 0.0f } };
        const Vector6 twist{ 0.3f, -0.1f, 0.2f, 0.5f, 0.4f, -0.3f };

        template<std::size_t Rows, std::size_t Cols>
        static void ExpectNear(const math::Matrix<float, Rows, Cols>& actual, const math::Matrix<float, Rows, Cols>& expected, float eps = tolerance)
        {
            for (std::size_t r = 0; r < Rows; ++r)
                for (std::size_t c = 0; c < Cols; ++c)
                    EXPECT_NEAR(actual.at(r, c), expected.at(r, c), eps) << "(" << r << ", " << c << ")";
        }

        static void ExpectNear(const Transform& actual, const Transform& expected, float eps = tolerance)
        {
            ExpectNear(actual.rotation, expected.rotation, eps);
            ExpectNear(actual.translation, expected.translation, eps);
        }

        static Vector3 UnitAxis(float x, float y, float z)
        {
            const float norm{ std::sqrt(x * x + y * y + z * z) };
            return Vector3{ x / norm, y / norm, z / norm };
        }

        static float Dot(const Vector6& lhs, const Vector6& rhs)
        {
            float dot{ 0.0f };
            for (std::size_t i = 0; i < 6; ++i)
                dot += lhs[i] * rhs[i];

            return dot;
        }
    };
}

TEST_F(TestSE3Transform, apply_rotates_then_translates)
{
    ExpectNear(a.Apply(xAxis), Vector3{ 1.0f, 1.0f, 0.0f });
}

TEST_F(TestSE3Transform, composition_applies_right_operand_first)
{
    const Vector3 x{ 0.3f, -0.2f, 0.5f };

    ExpectNear((a * b).Apply(x), a.Apply(b.Apply(x)));
}

TEST_F(TestSE3Transform, inverse_composes_to_identity)
{
    ExpectNear(a * a.Inverse(), Transform::Identity());
    ExpectNear(a.Inverse() * a, Transform::Identity());
}

TEST_F(TestSE3Transform, homogeneous_matches_block_form)
{
    const auto h{ a.Homogeneous() };

    ExpectNear(h.GetBlock<3, 3>(0, 0), a.rotation, 0.0f);
    ExpectNear(h.GetBlock<3, 1>(0, 3), a.translation, 0.0f);
    ExpectNear(h.GetBlock<1, 4>(3, 0), math::Matrix<float, 1, 4>{ 0.0f, 0.0f, 0.0f, 1.0f }, 0.0f);
}

TEST_F(TestSE3Transform, adjoint_maps_twists_consistently_with_composition)
{
    ExpectNear(Transform::Exp(a.TransformTwist(twist), 0.4f), a * Transform::Exp(twist, 0.4f) * a.Inverse());
    ExpectNear(a.TransformTwist(twist), Vector6{ a.Adjoint() * twist });
}

TEST_F(TestSE3Transform, adjoint_of_product_is_product_of_adjoints)
{
    ExpectNear((a * b).Adjoint(), a.Adjoint() * b.Adjoint());
}

TEST_F(TestSE3Transform, wrench_and_twist_power_is_frame_invariant)
{
    const Vector6 wrench{ 1.0f, -2.0f, 0.5f, 0.3f, 0.7f, -0.4f };

    EXPECT_NEAR(Dot(a.TransformWrench(wrench), a.TransformTwist(twist)), Dot(wrench, twist), tolerance);
}

TEST_F(TestSE3Transform, exp_of_pure_rotation_about_offset_axis)
{
    const Transform screw{ Transform::Exp(Vector6{ 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 1.0f }, pi / 2.0f) };

    ExpectNear(screw.translation, Vector3{ 1.0f, -1.0f, 0.0f });
    ExpectNear(screw.rotation, math::RotationAboutAxis(zAxis, pi / 2.0f));
}

TEST_F(TestSE3Transform, exp_of_pure_translation)
{
    ExpectNear(Transform::Exp(Vector6{ 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }, 0.3f), Transform{ Transform::Identity().rotation, Vector3{ 0.3f, 0.0f, 0.0f } });
}

TEST_F(TestSE3Transform, log_inverts_exp)
{
    const Transform nearHalfTurn{ math::RotationAboutAxis(UnitAxis(1.0f, -2.0f, 0.5f), 179.9f * pi / 180.0f), Vector3{ 0.5f, -1.0f, 2.0f } };

    ExpectNear(Transform::Exp(a.Log(), 1.0f), a);
    ExpectNear(Transform::Exp(b.Log(), 1.0f), b);
    ExpectNear(Transform::Exp(nearHalfTurn.Log(), 1.0f), nearHalfTurn, 1e-4f);
}

TEST_F(TestSE3Transform, log_near_zero_and_near_pi_stays_finite)
{
    const Vector3 axis{ UnitAxis(0.2f, 0.3f, -0.9f) };

    ExpectNear(Transform::LogRotation(math::RotationAboutAxis(axis, 1e-6f)), axis * 1e-6f, 1e-10f);
    ExpectNear(Transform::LogRotation(math::RotationAboutAxis(axis, pi - 1e-4f)), axis * (pi - 1e-4f), 1e-3f);
}

TEST_F(TestSE3Transform, pose_error_takes_the_short_way)
{
    const Transform target{ math::RotationAboutAxis(zAxis, 190.0f * pi / 180.0f) * b.rotation, b.translation };

    const Vector6 error{ Transform::PoseError(target, b) };

    ExpectNear(error.GetBlock<3, 1>(0, 0), Vector3{});
    ExpectNear(error.GetBlock<3, 1>(3, 0), Vector3{ 0.0f, 0.0f, -170.0f * pi / 180.0f }, 1e-4f);
}
