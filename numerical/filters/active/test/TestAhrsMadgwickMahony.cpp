#include "numerical/filters/active/AhrsMadgwickMahony.hpp"
#include "numerical/math/Tolerance.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    class TestAhrsFilter
        : public ::testing::Test
    {
    public:
        filters::AhrsFilter<float, filters::AhrsMode::Madgwick> madgwick{ 0.1f, 0.01f };
        filters::AhrsFilter<float, filters::AhrsMode::Mahony> mahony{ 0.5f, 0.01f };
        static constexpr float g{ 9.81f };
    };
}

TEST_F(TestAhrsFilter, level_static_converges_to_identity)
{
    math::Vector3<float> gyro{ { 0.0f }, { 0.0f }, { 0.0f } };
    math::Vector3<float> accel{ { 0.0f }, { 0.0f }, { g } };

    for (int i = 0; i < 2000; ++i)
        madgwick.UpdateImu(gyro, accel);

    math::Quaternion<float> orient{ madgwick.Orientation() };
    EXPECT_NEAR(orient.w, 1.0f, 1e-2f);
    EXPECT_NEAR(orient.x, 0.0f, 1e-2f);
    EXPECT_NEAR(orient.y, 0.0f, 1e-2f);
    EXPECT_NEAR(orient.z, 0.0f, 1e-2f);
}

TEST_F(TestAhrsFilter, tilt_recovered_from_gravity)
{
    constexpr float rollDeg{ 30.0f };
    constexpr float rollRad{ rollDeg * std::numbers::pi_v<float> / 180.0f };

    float cr{ std::cos(rollRad) };
    float sr{ std::sin(rollRad) };
    math::Vector3<float> gyro{ { 0.0f }, { 0.0f }, { 0.0f } };
    math::Vector3<float> accel{ { 0.0f }, { g * sr }, { g * cr } };

    for (int i = 0; i < 3000; ++i)
        madgwick.UpdateImu(gyro, accel);

    math::Vector3<float> euler{ madgwick.Euler() };
    float rollResult{ euler.at(0, 0) * 180.0f / std::numbers::pi_v<float> };
    EXPECT_NEAR(rollResult, rollDeg, 1.0f);
    EXPECT_NEAR(euler.at(1, 0), 0.0f, 1e-2f);
}

TEST_F(TestAhrsFilter, pure_gyro_integrates_rotation)
{
    constexpr float omegaZ{ 1.0f };
    constexpr int n{ 50 };
    math::Vector3<float> gyro{ { 0.0f }, { 0.0f }, { omegaZ } };
    math::Vector3<float> accel{ { 0.0f }, { 0.0f }, { g } };

    for (int i = 0; i < n; ++i)
        madgwick.UpdateImu(gyro, accel);

    math::Vector3<float> euler{ madgwick.Euler() };
    float yaw{ euler.at(2, 0) };
    EXPECT_GT(yaw, 0.0f);
    EXPECT_LT(yaw, omegaZ * 0.01f * static_cast<float>(n) + 0.1f);
}

TEST_F(TestAhrsFilter, quaternion_stays_unit_norm)
{
    math::Vector3<float> gyro{ { 0.3f }, { -0.2f }, { 0.15f } };
    math::Vector3<float> accel{ { 0.5f }, { 0.1f }, { g } };

    for (int i = 0; i < 500; ++i)
    {
        madgwick.UpdateImu(gyro, accel);
        math::Quaternion<float> orient{ madgwick.Orientation() };
        float norm{ orient.Norm() };
        EXPECT_NEAR(norm, 1.0f, 1e-4f);
    }
}

TEST_F(TestAhrsFilter, zero_accel_skips_correction)
{
    math::Vector3<float> gyro{ { 0.0f }, { 0.0f }, { 1.0f } };
    math::Vector3<float> accel{ { 0.0f }, { 0.0f }, { 0.0f } };

    for (int i = 0; i < 100; ++i)
        madgwick.UpdateImu(gyro, accel);

    math::Quaternion<float> orient{ madgwick.Orientation() };
    EXPECT_FALSE(std::isnan(orient.w));
    EXPECT_FALSE(std::isnan(orient.x));
    EXPECT_FALSE(std::isnan(orient.y));
    EXPECT_FALSE(std::isnan(orient.z));
}

TEST_F(TestAhrsFilter, marg_constrains_yaw)
{
    math::Vector3<float> gyro{ { 0.0f }, { 0.0f }, { 0.0f } };
    math::Vector3<float> accel{ { 0.0f }, { 0.0f }, { g } };
    math::Vector3<float> mag{ { 1.0f }, { 0.0f }, { 0.0f } };

    for (int i = 0; i < 3000; ++i)
        madgwick.UpdateMarg(gyro, accel, mag);

    math::Vector3<float> euler{ madgwick.Euler() };
    EXPECT_NEAR(euler.at(2, 0), 0.0f, 1e-1f);
}

TEST_F(TestAhrsFilter, mahony_estimates_gyro_bias)
{
    constexpr float biasMag{ 0.05f };
    math::Vector3<float> gyro{ { biasMag }, { 0.0f }, { 0.0f } };
    math::Vector3<float> accel{ { 0.0f }, { 0.0f }, { g } };

    for (int i = 0; i < 5000; ++i)
        mahony.UpdateImu(gyro, accel);

    math::Quaternion<float> orient{ mahony.Orientation() };
    EXPECT_NEAR(orient.w, 1.0f, 1e-1f);
    EXPECT_NEAR(orient.x, 0.0f, 1e-1f);
    EXPECT_NEAR(orient.y, 0.0f, 1e-1f);
}

TEST_F(TestAhrsFilter, madgwick_and_mahony_agree_static)
{
    math::Vector3<float> gyro{ { 0.0f }, { 0.0f }, { 0.0f } };
    math::Vector3<float> accel{ { 0.0f }, { 0.0f }, { g } };

    for (int i = 0; i < 3000; ++i)
    {
        madgwick.UpdateImu(gyro, accel);
        mahony.UpdateImu(gyro, accel);
    }

    math::Quaternion<float> qm{ madgwick.Orientation() };
    math::Quaternion<float> qn{ mahony.Orientation() };
    EXPECT_NEAR(qm.w, qn.w, 1e-2f);
    EXPECT_NEAR(qm.x, qn.x, 1e-2f);
    EXPECT_NEAR(qm.y, qn.y, 1e-2f);
    EXPECT_NEAR(std::abs(qm.z), std::abs(qn.z), 1e-2f);
}

TEST_F(TestAhrsFilter, reset_restores_identity)
{
    math::Vector3<float> gyro{ { 0.5f }, { 0.3f }, { -0.2f } };
    math::Vector3<float> accel{ { 0.5f }, { 0.1f }, { g } };

    for (int i = 0; i < 100; ++i)
        madgwick.UpdateImu(gyro, accel);

    madgwick.Reset();

    math::Quaternion<float> orient{ madgwick.Orientation() };
    EXPECT_NEAR(orient.w, 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(orient.x, 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(orient.y, 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(orient.z, 0.0f, math::Tolerance<float>());
}
