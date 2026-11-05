#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/SE2Transform.hpp"
#include <cmath>
#include <cstddef>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    using Transform = math::SE2Transform<float>;
    using Vector2 = math::Vector2<float>;
    using Vector3 = math::Vector3<float>;

    class TestSE2Transform : public ::testing::Test
    {
    protected:
        static constexpr float pi{ std::numbers::pi_v<float> };
        static constexpr float tolerance{ 1e-5f };
        const Transform a{ Transform::FromPose(1.0f, 2.0f, pi / 2.0f) };
        const Transform b{ Transform::FromPose(-0.5f, 0.3f, -pi / 3.0f) };
        const Vector3 twist{ 0.4f, -0.2f, 0.7f };

        template<std::size_t Rows, std::size_t Cols>
        static void ExpectNear(const math::Matrix<float, Rows, Cols>& actual, const math::Matrix<float, Rows, Cols>& expected, float eps = tolerance)
        {
            for (std::size_t r = 0; r < Rows; ++r)
                for (std::size_t c = 0; c < Cols; ++c)
                    EXPECT_NEAR(actual.at(r, c), expected.at(r, c), eps) << "(" << r << ", " << c << ")";
        }

        static void ExpectNear(const Transform& actual, const Transform& expected, float eps = tolerance)
        {
            EXPECT_NEAR(actual.cosine, expected.cosine, eps);
            EXPECT_NEAR(actual.sine, expected.sine, eps);
            ExpectNear(actual.translation, expected.translation, eps);
        }

        static float Dot(const Vector3& lhs, const Vector3& rhs)
        {
            return lhs[0] * rhs[0] + lhs[1] * rhs[1] + lhs[2] * rhs[2];
        }

        static Vector2 ClosedFormTranslation(float phi, const Vector2& u)
        {
            const float sinc{ std::sin(phi) / phi };
            const float versinc{ (1.0f - std::cos(phi)) / phi };
            return Vector2{ sinc * u[0] - versinc * u[1], versinc * u[0] + sinc * u[1] };
        }
    };
}

TEST_F(TestSE2Transform, apply_rotates_then_translates)
{
    ExpectNear(a.Apply(Vector2{ 1.0f, 0.0f }), Vector2{ 1.0f, 3.0f });
}

TEST_F(TestSE2Transform, composition_applies_right_operand_first)
{
    const Vector2 x{ 0.3f, -0.2f };

    ExpectNear((a * b).Apply(x), a.Apply(b.Apply(x)));
    ExpectNear((a * b).Pose(), Vector3{ 0.7f, 1.5f, pi / 6.0f });
}

TEST_F(TestSE2Transform, inverse_composes_to_identity)
{
    ExpectNear(a.Inverse().Pose(), Vector3{ -2.0f, 1.0f, -pi / 2.0f });
    ExpectNear(a * a.Inverse(), Transform::Identity());
    ExpectNear(a.Inverse() * a, Transform::Identity());
}

TEST_F(TestSE2Transform, pose_returns_wrapped_heading)
{
    EXPECT_NEAR(Transform::FromPose(0.0f, 0.0f, 3.0f * pi / 2.0f).Heading(), -pi / 2.0f, tolerance);
    ExpectNear(b.Pose(), Vector3{ -0.5f, 0.3f, -pi / 3.0f });
}

TEST_F(TestSE2Transform, homogeneous_matches_block_form)
{
    ExpectNear(a.Homogeneous(), math::SquareMatrix<float, 3>{ { 0.0f, -1.0f, 1.0f }, { 1.0f, 0.0f, 2.0f }, { 0.0f, 0.0f, 1.0f } });
    ExpectNear(a.Rotation(), math::Matrix2<float>{ { 0.0f, -1.0f }, { 1.0f, 0.0f } });
}

TEST_F(TestSE2Transform, adjoint_maps_twists_consistently_with_composition)
{
    for (const float t : { 0.4f, 2.0f })
        ExpectNear(Transform::Exp(a.TransformTwist(twist), t), a * Transform::Exp(twist, t) * a.Inverse());
}

TEST_F(TestSE2Transform, adjoint_of_product_is_product_of_adjoints)
{
    ExpectNear((a * b).Adjoint(), math::Matrix3<float>{ a.Adjoint() * b.Adjoint() });
}

TEST_F(TestSE2Transform, wrench_and_twist_power_is_frame_invariant)
{
    const Vector3 wrench{ 1.5f, -0.7f, 0.3f };

    EXPECT_NEAR(Dot(a.TransformWrench(wrench), a.TransformTwist(twist)), Dot(wrench, twist), tolerance);
    EXPECT_NEAR(Dot(wrench, twist), 0.95f, tolerance);
}

TEST_F(TestSE2Transform, exp_of_unicycle_arc)
{
    ExpectNear(Transform::Exp(Vector3{ 1.0f, 0.0f, 1.0f }, pi / 2.0f).Pose(), Vector3{ 1.0f, 1.0f, pi / 2.0f });
}

TEST_F(TestSE2Transform, exp_of_pure_translation)
{
    ExpectNear(Transform::Exp(Vector3{ 0.3f, -0.4f, 0.0f }, 2.0f).Pose(), Vector3{ 0.6f, -0.8f, 0.0f });
}

TEST_F(TestSE2Transform, exp_is_continuous_across_series_threshold)
{
    const Vector2 linear{ 0.8f, 0.3f };

    for (const float phi : { 0.1f - 1e-4f, 0.1f + 1e-4f })
        ExpectNear(Transform::Exp(Vector3{ linear[0], linear[1], phi }, 1.0f).translation, ClosedFormTranslation(phi, linear), 1e-6f);
}

TEST_F(TestSE2Transform, log_inverts_exp)
{
    ExpectNear(a.Log(), Vector3{ 3.0f * pi / 4.0f, pi / 4.0f, pi / 2.0f });
    for (const auto& transform : { a, b, Transform::FromPose(0.2f, -0.4f, 179.9f * pi / 180.0f) })
        ExpectNear(Transform::Exp(transform.Log(), 1.0f), transform);
}

TEST_F(TestSE2Transform, log_at_half_turn_is_finite)
{
    const Transform halfTurn{ -1.0f, 0.0f, Vector2{ 1.0f, 0.0f } };

    ExpectNear(halfTurn.Log(), Vector3{ 0.0f, -pi / 2.0f, pi });
}

TEST_F(TestSE2Transform, pose_error_takes_the_short_way)
{
    const Transform current{ Transform::FromPose(1.0f, 1.0f, 0.2f) };
    const Transform target{ Transform::FromPose(2.0f, 0.5f, 0.2f + 190.0f * pi / 180.0f) };

    ExpectNear(Transform::PoseError(target, current), Vector3{ 1.0f, -0.5f, -2.967060f });
}

TEST_F(TestSE2Transform, normalized_restores_unit_rotation)
{
    const Transform scaled{ 1.001f * std::cos(0.3f), 1.001f * std::sin(0.3f), Vector2{ 1.0f, 2.0f } };

    const auto normalized{ scaled.Normalized() };

    EXPECT_NEAR(normalized.cosine * normalized.cosine + normalized.sine * normalized.sine, 1.0f, tolerance);
    EXPECT_NEAR(normalized.Heading(), 0.3f, tolerance);
    ExpectNear(normalized.translation, Vector2{ 1.0f, 2.0f });
}

TEST_F(TestSE2Transform, to_se3_rotates_about_z)
{
    const auto lifted{ a.ToSE3() };

    ExpectNear(lifted.rotation, math::RotationAboutAxis(Vector3{ 0.0f, 0.0f, 1.0f }, pi / 2.0f));
    ExpectNear(lifted.translation, Vector3{ 1.0f, 2.0f, 0.0f });
}
