#include "numerical/math/Quaternion.hpp"
#include "numerical/math/Tolerance.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    class TestQuaternion
        : public ::testing::Test
    {
    protected:
        math::Quaternion<float> identity{ math::Quaternion<float>::Identity() };
    };
}

TEST_F(TestQuaternion, IdentityIsNeutralProduct)
{
    auto q = math::Quaternion<float>::FromAxisAngle(
        math::Vector3<float>{ { 0.0f }, { 0.0f }, { 1.0f } }, 0.5f);
    auto r = q * identity;
    EXPECT_NEAR(r.w, q.w, math::Tolerance<float>());
    EXPECT_NEAR(r.x, q.x, math::Tolerance<float>());
    EXPECT_NEAR(r.y, q.y, math::Tolerance<float>());
    EXPECT_NEAR(r.z, q.z, math::Tolerance<float>());
}

TEST_F(TestQuaternion, HamiltonProductNoncommutative)
{
    auto qx = math::Quaternion<float>::FromAxisAngle(
        math::Vector3<float>{ { 1.0f }, { 0.0f }, { 0.0f } },
        std::numbers::pi_v<float> / 2.0f);
    auto qy = math::Quaternion<float>::FromAxisAngle(
        math::Vector3<float>{ { 0.0f }, { 1.0f }, { 0.0f } },
        std::numbers::pi_v<float> / 2.0f);
    auto qxy = qx * qy;
    auto qyx = qy * qx;
    bool nonCommutative = (std::abs(qxy.w - qyx.w) > math::Tolerance<float>()) ||
                          (std::abs(qxy.x - qyx.x) > math::Tolerance<float>()) ||
                          (std::abs(qxy.y - qyx.y) > math::Tolerance<float>()) ||
                          (std::abs(qxy.z - qyx.z) > math::Tolerance<float>());
    EXPECT_TRUE(nonCommutative);
}

TEST_F(TestQuaternion, ConjugateInvertsUnitRotation)
{
    auto q = math::Quaternion<float>::FromAxisAngle(
        math::Vector3<float>{ { 0.0f }, { 0.0f }, { 1.0f } }, 0.7f);
    auto r = q * q.Conjugate();
    EXPECT_NEAR(r.w, 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(r.x, 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(r.y, 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(r.z, 0.0f, math::Tolerance<float>());
}

TEST_F(TestQuaternion, RotateVectorMatchesMatrix)
{
    auto q = math::Quaternion<float>::FromAxisAngle(
        math::Vector3<float>{ { 0.0f }, { 0.0f }, { 1.0f } },
        std::numbers::pi_v<float> / 3.0f);
    math::Vector3<float> v{ { 1.0f }, { 0.0f }, { 0.0f } };
    auto rotByQ = q.Rotate(v);
    auto r = q.ToRotationMatrix();
    auto rotByM = r * v;
    EXPECT_NEAR(rotByQ.at(0, 0), rotByM.at(0, 0), math::Tolerance<float>());
    EXPECT_NEAR(rotByQ.at(1, 0), rotByM.at(1, 0), math::Tolerance<float>());
    EXPECT_NEAR(rotByQ.at(2, 0), rotByM.at(2, 0), math::Tolerance<float>());
}

TEST_F(TestQuaternion, AxisAngleRoundTrip)
{
    auto q = math::Quaternion<float>::FromAxisAngle(
        math::Vector3<float>{ { 0.0f }, { 0.0f }, { 1.0f } },
        std::numbers::pi_v<float> / 2.0f);
    math::Vector3<float> xHat{ { 1.0f }, { 0.0f }, { 0.0f } };
    auto result = q.Rotate(xHat);
    EXPECT_NEAR(result.at(0, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.at(1, 0), 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.at(2, 0), 0.0f, math::Tolerance<float>());
}

TEST_F(TestQuaternion, MatrixRoundTrip)
{
    auto q = math::Quaternion<float>::FromAxisAngle(
        math::Vector3<float>{ { 0.0f }, { 0.0f }, { 1.0f } },
        std::numbers::pi_v<float> / 4.0f);
    auto r = q.ToRotationMatrix();
    auto qPrime = math::Quaternion<float>::FromRotationMatrix(r);
    bool sameRotation =
        (std::abs(qPrime.w - q.w) < math::Tolerance<float>() && std::abs(qPrime.x - q.x) < math::Tolerance<float>() &&
            std::abs(qPrime.y - q.y) < math::Tolerance<float>() && std::abs(qPrime.z - q.z) < math::Tolerance<float>()) ||
        (std::abs(qPrime.w + q.w) < math::Tolerance<float>() && std::abs(qPrime.x + q.x) < math::Tolerance<float>() &&
            std::abs(qPrime.y + q.y) < math::Tolerance<float>() && std::abs(qPrime.z + q.z) < math::Tolerance<float>());
    EXPECT_TRUE(sameRotation);
}

TEST_F(TestQuaternion, SlerpEndpoints)
{
    auto a = math::Quaternion<float>::FromAxisAngle(
        math::Vector3<float>{ { 1.0f }, { 0.0f }, { 0.0f } }, 0.3f);
    auto b = math::Quaternion<float>::FromAxisAngle(
        math::Vector3<float>{ { 0.0f }, { 1.0f }, { 0.0f } }, 0.9f);
    auto s0 = math::Quaternion<float>::Slerp(a, b, 0.0f);
    auto s1 = math::Quaternion<float>::Slerp(a, b, 1.0f);
    EXPECT_NEAR(s0.w, a.w, math::Tolerance<float>());
    EXPECT_NEAR(s0.x, a.x, math::Tolerance<float>());
    EXPECT_NEAR(s0.y, a.y, math::Tolerance<float>());
    EXPECT_NEAR(s0.z, a.z, math::Tolerance<float>());
    EXPECT_NEAR(s1.w, b.w, math::Tolerance<float>());
    EXPECT_NEAR(s1.x, b.x, math::Tolerance<float>());
    EXPECT_NEAR(s1.y, b.y, math::Tolerance<float>());
    EXPECT_NEAR(s1.z, b.z, math::Tolerance<float>());
}

TEST_F(TestQuaternion, SlerpMidpointConstantSpeed)
{
    auto a = math::Quaternion<float>::Identity();
    auto b = math::Quaternion<float>::FromAxisAngle(
        math::Vector3<float>{ { 0.0f }, { 0.0f }, { 1.0f } },
        std::numbers::pi_v<float> / 2.0f);
    auto mid = math::Quaternion<float>::Slerp(a, b, 0.5f);
    auto expected = math::Quaternion<float>::FromAxisAngle(
        math::Vector3<float>{ { 0.0f }, { 0.0f }, { 1.0f } },
        std::numbers::pi_v<float> / 4.0f);
    EXPECT_NEAR(mid.w, expected.w, math::Tolerance<float>());
    EXPECT_NEAR(mid.x, expected.x, math::Tolerance<float>());
    EXPECT_NEAR(mid.y, expected.y, math::Tolerance<float>());
    EXPECT_NEAR(mid.z, expected.z, math::Tolerance<float>());
}

TEST_F(TestQuaternion, NormalizeRestoresUnitNorm)
{
    auto q = math::Quaternion<float>::FromAxisAngle(
        math::Vector3<float>{ { 0.0f }, { 0.0f }, { 1.0f } }, 0.5f);
    math::Quaternion<float> scaled{ q.w * 1.1f, q.x * 1.1f, q.y * 1.1f, q.z * 1.1f };
    scaled.Normalize();
    EXPECT_NEAR(scaled.Norm(), 1.0f, math::Tolerance<float>());
}

TEST_F(TestQuaternion, EulerRoundTrip)
{
    float roll{ 0.3f };
    float pitch{ 0.2f };
    float yaw{ 0.5f };
    auto q = math::Quaternion<float>::FromEulerZYX(roll, pitch, yaw);
    auto euler = q.ToEulerZYX();
    EXPECT_NEAR(euler.at(0, 0), roll, math::Tolerance<float>());
    EXPECT_NEAR(euler.at(1, 0), pitch, math::Tolerance<float>());
    EXPECT_NEAR(euler.at(2, 0), yaw, math::Tolerance<float>());
}

TEST_F(TestQuaternion, UnaryNegation)
{
    auto q = math::Quaternion<float>::FromAxisAngle(
        math::Vector3<float>{ { 0.0f }, { 0.0f }, { 1.0f } }, 0.5f);
    auto neg = -q;
    EXPECT_NEAR(neg.w, -q.w, math::Tolerance<float>());
    EXPECT_NEAR(neg.x, -q.x, math::Tolerance<float>());
    EXPECT_NEAR(neg.y, -q.y, math::Tolerance<float>());
    EXPECT_NEAR(neg.z, -q.z, math::Tolerance<float>());
}

TEST_F(TestQuaternion, InverseOfUnitQuaternion)
{
    auto q = math::Quaternion<float>::FromAxisAngle(
        math::Vector3<float>{ { 1.0f }, { 0.0f }, { 0.0f } }, 0.8f);
    auto r = q * q.Inverse();
    EXPECT_NEAR(r.w, 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(r.x, 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(r.y, 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(r.z, 0.0f, math::Tolerance<float>());
}

TEST_F(TestQuaternion, SquaredNormOfUnitQuaternion)
{
    auto q = math::Quaternion<float>::FromAxisAngle(
        math::Vector3<float>{ { 0.0f }, { 1.0f }, { 0.0f } }, 1.2f);
    EXPECT_NEAR(q.SquaredNorm(), 1.0f, math::Tolerance<float>());
}

TEST_F(TestQuaternion, NormalizedReturnsNewCopy)
{
    math::Quaternion<float> scaled{ 2.0f, 0.0f, 0.0f, 0.0f };
    auto n = scaled.Normalized();
    EXPECT_NEAR(n.Norm(), 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(scaled.Norm(), 2.0f, math::Tolerance<float>());
}

TEST_F(TestQuaternion, FromRotationMatrixXDominant)
{
    auto q = math::Quaternion<float>::FromAxisAngle(
        math::Vector3<float>{ { 1.0f }, { 0.0f }, { 0.0f } },
        std::numbers::pi_v<float>);
    auto r = q.ToRotationMatrix();
    auto qPrime = math::Quaternion<float>::FromRotationMatrix(r);
    bool sameRotation =
        (std::abs(qPrime.w - q.w) < math::Tolerance<float>() && std::abs(qPrime.x - q.x) < math::Tolerance<float>() &&
            std::abs(qPrime.y - q.y) < math::Tolerance<float>() && std::abs(qPrime.z - q.z) < math::Tolerance<float>()) ||
        (std::abs(qPrime.w + q.w) < math::Tolerance<float>() && std::abs(qPrime.x + q.x) < math::Tolerance<float>() &&
            std::abs(qPrime.y + q.y) < math::Tolerance<float>() && std::abs(qPrime.z + q.z) < math::Tolerance<float>());
    EXPECT_TRUE(sameRotation);
}

TEST_F(TestQuaternion, FromRotationMatrixYDominant)
{
    auto q = math::Quaternion<float>::FromAxisAngle(
        math::Vector3<float>{ { 0.0f }, { 1.0f }, { 0.0f } },
        std::numbers::pi_v<float>);
    auto r = q.ToRotationMatrix();
    auto qPrime = math::Quaternion<float>::FromRotationMatrix(r);
    bool sameRotation =
        (std::abs(qPrime.w - q.w) < math::Tolerance<float>() && std::abs(qPrime.x - q.x) < math::Tolerance<float>() &&
            std::abs(qPrime.y - q.y) < math::Tolerance<float>() && std::abs(qPrime.z - q.z) < math::Tolerance<float>()) ||
        (std::abs(qPrime.w + q.w) < math::Tolerance<float>() && std::abs(qPrime.x + q.x) < math::Tolerance<float>() &&
            std::abs(qPrime.y + q.y) < math::Tolerance<float>() && std::abs(qPrime.z + q.z) < math::Tolerance<float>());
    EXPECT_TRUE(sameRotation);
}

TEST_F(TestQuaternion, FromRotationMatrixZDominant)
{
    auto q = math::Quaternion<float>::FromAxisAngle(
        math::Vector3<float>{ { 0.0f }, { 0.0f }, { 1.0f } },
        std::numbers::pi_v<float>);
    auto r = q.ToRotationMatrix();
    auto qPrime = math::Quaternion<float>::FromRotationMatrix(r);
    bool sameRotation =
        (std::abs(qPrime.w - q.w) < math::Tolerance<float>() && std::abs(qPrime.x - q.x) < math::Tolerance<float>() &&
            std::abs(qPrime.y - q.y) < math::Tolerance<float>() && std::abs(qPrime.z - q.z) < math::Tolerance<float>()) ||
        (std::abs(qPrime.w + q.w) < math::Tolerance<float>() && std::abs(qPrime.x + q.x) < math::Tolerance<float>() &&
            std::abs(qPrime.y + q.y) < math::Tolerance<float>() && std::abs(qPrime.z + q.z) < math::Tolerance<float>());
    EXPECT_TRUE(sameRotation);
}

TEST_F(TestQuaternion, SlerpAntipodalFlipsSignAndMatchesDirectPath)
{
    auto a = math::Quaternion<float>::FromAxisAngle(
        math::Vector3<float>{ { 1.0f }, { 0.0f }, { 0.0f } }, 0.4f);
    auto b = math::Quaternion<float>::FromAxisAngle(
        math::Vector3<float>{ { 0.0f }, { 0.0f }, { 1.0f } }, 0.6f);
    auto s1 = math::Quaternion<float>::Slerp(a, b, 0.5f);
    auto s2 = math::Quaternion<float>::Slerp(a, -b, 0.5f);
    bool sameRotation =
        (std::abs(s1.w - s2.w) < math::Tolerance<float>() && std::abs(s1.x - s2.x) < math::Tolerance<float>() &&
            std::abs(s1.y - s2.y) < math::Tolerance<float>() && std::abs(s1.z - s2.z) < math::Tolerance<float>()) ||
        (std::abs(s1.w + s2.w) < math::Tolerance<float>() && std::abs(s1.x + s2.x) < math::Tolerance<float>() &&
            std::abs(s1.y + s2.y) < math::Tolerance<float>() && std::abs(s1.z + s2.z) < math::Tolerance<float>());
    EXPECT_TRUE(sameRotation);
}

TEST_F(TestQuaternion, SlerpNearParallelFallsBackToLerp)
{
    auto a = math::Quaternion<float>::Identity();
    auto b = math::Quaternion<float>::FromAxisAngle(
        math::Vector3<float>{ { 0.0f }, { 0.0f }, { 1.0f } }, 1e-4f);
    auto mid = math::Quaternion<float>::Slerp(a, b, 0.5f);
    auto expected = math::Quaternion<float>::FromAxisAngle(
        math::Vector3<float>{ { 0.0f }, { 0.0f }, { 1.0f } }, 5e-5f);
    EXPECT_NEAR(mid.w, expected.w, math::Tolerance<float>());
    EXPECT_NEAR(mid.z, expected.z, math::Tolerance<float>());
}

TEST_F(TestQuaternion, ToEulerGimbalLockPositivePitch)
{
    // w=0.72, y=0.72 → sinP = 2*0.72*0.72 = 1.0368 > 1, forcing copysign branch
    math::Quaternion<float> q{ 0.72f, 0.0f, 0.72f, 0.0f };
    auto euler = q.ToEulerZYX();
    EXPECT_NEAR(euler.at(1, 0), std::numbers::pi_v<float> / 2.0f, math::Tolerance<float>());
}

TEST_F(TestQuaternion, ToEulerGimbalLockNegativePitch)
{
    // w=0.72, y=-0.72 → sinP = -1.0368 < -1, forcing copysign with negative sign
    math::Quaternion<float> q{ 0.72f, 0.0f, -0.72f, 0.0f };
    auto euler = q.ToEulerZYX();
    EXPECT_NEAR(euler.at(1, 0), -std::numbers::pi_v<float> / 2.0f, math::Tolerance<float>());
}
