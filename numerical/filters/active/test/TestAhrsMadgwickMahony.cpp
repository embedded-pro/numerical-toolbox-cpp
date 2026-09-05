#include "numerical/filters/active/AhrsMadgwickMahony.hpp"
#include "numerical/math/Tolerance.hpp"
#include <array>
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

TEST_F(TestAhrsFilter, mahony_marg_constrains_yaw)
{
    math::Vector3<float> gyro{ { 0.0f }, { 0.0f }, { 0.0f } };
    math::Vector3<float> accel{ { 0.0f }, { 0.0f }, { g } };
    math::Vector3<float> mag{ { 1.0f }, { 0.0f }, { 0.0f } };

    for (int i = 0; i < 3000; ++i)
        mahony.UpdateMarg(gyro, accel, mag);

    math::Vector3<float> euler{ mahony.Euler() };
    EXPECT_NEAR(euler.at(2, 0), 0.0f, 1e-1f);
}

TEST_F(TestAhrsFilter, marg_zero_mag_falls_back_to_imu)
{
    constexpr float rollDeg{ 30.0f };
    constexpr float rollRad{ rollDeg * std::numbers::pi_v<float> / 180.0f };

    float cr{ std::cos(rollRad) };
    float sr{ std::sin(rollRad) };
    math::Vector3<float> gyro{ { 0.0f }, { 0.0f }, { 0.0f } };
    math::Vector3<float> accel{ { 0.0f }, { g * sr }, { g * cr } };
    math::Vector3<float> mag{ { 0.0f }, { 0.0f }, { 0.0f } };

    for (int i = 0; i < 3000; ++i)
        madgwick.UpdateMarg(gyro, accel, mag);

    math::Vector3<float> euler{ madgwick.Euler() };
    EXPECT_NEAR(euler.at(0, 0) * 180.0f / std::numbers::pi_v<float>, rollDeg, 1.0f);
}

TEST_F(TestAhrsFilter, marg_zero_accel_skips_correction)
{
    math::Vector3<float> gyro{ { 0.0f }, { 0.0f }, { 1.0f } };
    math::Vector3<float> accel{ { 0.0f }, { 0.0f }, { 0.0f } };
    math::Vector3<float> mag{ { 1.0f }, { 0.0f }, { 0.0f } };

    for (int i = 0; i < 100; ++i)
        madgwick.UpdateMarg(gyro, accel, mag);

    math::Quaternion<float> orient{ madgwick.Orientation() };
    EXPECT_NEAR(orient.Norm(), 1.0f, 1e-4f);
    EXPECT_FALSE(std::isnan(orient.w));
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

TEST_F(TestAhrsFilter, mahony_level_static_converges_to_identity)
{
    math::Vector3<float> gyro{ { 0.0f }, { 0.0f }, { 0.0f } };
    math::Vector3<float> accel{ { 0.0f }, { 0.0f }, { g } };

    for (int i = 0; i < 3000; ++i)
        mahony.UpdateImu(gyro, accel);

    math::Quaternion<float> orient{ mahony.Orientation() };
    EXPECT_NEAR(orient.w, 1.0f, 1e-2f);
    EXPECT_NEAR(orient.x, 0.0f, 1e-2f);
    EXPECT_NEAR(orient.y, 0.0f, 1e-2f);
    EXPECT_NEAR(orient.z, 0.0f, 1e-2f);
}

TEST_F(TestAhrsFilter, mahony_tilt_recovered_from_gravity)
{
    constexpr float rollDeg{ 30.0f };
    constexpr float rollRad{ rollDeg * std::numbers::pi_v<float> / 180.0f };

    float cr{ std::cos(rollRad) };
    float sr{ std::sin(rollRad) };
    math::Vector3<float> gyro{ { 0.0f }, { 0.0f }, { 0.0f } };
    math::Vector3<float> accel{ { 0.0f }, { g * sr }, { g * cr } };

    for (int i = 0; i < 5000; ++i)
        mahony.UpdateImu(gyro, accel);

    math::Vector3<float> euler{ mahony.Euler() };
    float rollResult{ euler.at(0, 0) * 180.0f / std::numbers::pi_v<float> };
    EXPECT_NEAR(rollResult, rollDeg, 1.0f);
    EXPECT_NEAR(euler.at(1, 0), 0.0f, 1e-2f);
}

TEST_F(TestAhrsFilter, mahony_quaternion_stays_unit_norm)
{
    math::Vector3<float> gyro{ { 0.3f }, { -0.2f }, { 0.15f } };
    math::Vector3<float> accel{ { 0.5f }, { 0.1f }, { g } };

    for (int i = 0; i < 500; ++i)
    {
        mahony.UpdateImu(gyro, accel);
        float norm{ mahony.Orientation().Norm() };
        EXPECT_NEAR(norm, 1.0f, 1e-4f);
    }
}

TEST_F(TestAhrsFilter, mahony_zero_accel_skips_correction)
{
    math::Vector3<float> gyro{ { 0.0f }, { 0.0f }, { 1.0f } };
    math::Vector3<float> accel{ { 0.0f }, { 0.0f }, { 0.0f } };

    for (int i = 0; i < 100; ++i)
        mahony.UpdateImu(gyro, accel);

    math::Quaternion<float> orient{ mahony.Orientation() };
    EXPECT_FALSE(std::isnan(orient.w));
    EXPECT_FALSE(std::isnan(orient.x));
    EXPECT_FALSE(std::isnan(orient.y));
    EXPECT_FALSE(std::isnan(orient.z));
    EXPECT_NEAR(orient.Norm(), 1.0f, 1e-4f);
}

TEST_F(TestAhrsFilter, mahony_reset_restores_identity)
{
    math::Vector3<float> gyro{ { 0.5f }, { 0.3f }, { -0.2f } };
    math::Vector3<float> accel{ { 0.5f }, { 0.1f }, { g } };

    for (int i = 0; i < 100; ++i)
        mahony.UpdateImu(gyro, accel);

    mahony.Reset();

    math::Quaternion<float> orient{ mahony.Orientation() };
    EXPECT_NEAR(orient.w, 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(orient.x, 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(orient.y, 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(orient.z, 0.0f, math::Tolerance<float>());
}

TEST_F(TestAhrsFilter, madgwick_determinism)
{
    filters::AhrsFilter<float, filters::AhrsMode::Madgwick> a{ 0.1f, 0.01f };
    filters::AhrsFilter<float, filters::AhrsMode::Madgwick> b{ 0.1f, 0.01f };

    math::Vector3<float> gyro{ { 0.3f }, { -0.1f }, { 0.2f } };
    math::Vector3<float> accel{ { 0.5f }, { 0.4f }, { g } };

    for (int i = 0; i < 200; ++i)
    {
        a.UpdateImu(gyro, accel);
        b.UpdateImu(gyro, accel);
    }

    math::Quaternion<float> qa{ a.Orientation() };
    math::Quaternion<float> qb{ b.Orientation() };
    EXPECT_FLOAT_EQ(qa.w, qb.w);
    EXPECT_FLOAT_EQ(qa.x, qb.x);
    EXPECT_FLOAT_EQ(qa.y, qb.y);
    EXPECT_FLOAT_EQ(qa.z, qb.z);
}

TEST_F(TestAhrsFilter, mahony_determinism)
{
    filters::AhrsFilter<float, filters::AhrsMode::Mahony> a{ 0.5f, 0.01f };
    filters::AhrsFilter<float, filters::AhrsMode::Mahony> b{ 0.5f, 0.01f };

    math::Vector3<float> gyro{ { 0.3f }, { -0.1f }, { 0.2f } };
    math::Vector3<float> accel{ { 0.5f }, { 0.4f }, { g } };

    for (int i = 0; i < 200; ++i)
    {
        a.UpdateImu(gyro, accel);
        b.UpdateImu(gyro, accel);
    }

    math::Quaternion<float> qa{ a.Orientation() };
    math::Quaternion<float> qb{ b.Orientation() };
    EXPECT_FLOAT_EQ(qa.w, qb.w);
    EXPECT_FLOAT_EQ(qa.x, qb.x);
    EXPECT_FLOAT_EQ(qa.y, qb.y);
    EXPECT_FLOAT_EQ(qa.z, qb.z);
}

TEST_F(TestAhrsFilter, madgwick_reset_then_rerun_matches_fresh_instance)
{
    math::Vector3<float> gyro{ { 0.2f }, { -0.1f }, { 0.3f } };
    math::Vector3<float> accel{ { 0.3f }, { 0.2f }, { g } };

    for (int i = 0; i < 100; ++i)
        madgwick.UpdateImu(gyro, accel);

    madgwick.Reset();

    filters::AhrsFilter<float, filters::AhrsMode::Madgwick> fresh{ 0.1f, 0.01f };

    for (int i = 0; i < 50; ++i)
    {
        madgwick.UpdateImu(gyro, accel);
        fresh.UpdateImu(gyro, accel);
    }

    math::Quaternion<float> qr{ madgwick.Orientation() };
    math::Quaternion<float> qf{ fresh.Orientation() };
    EXPECT_FLOAT_EQ(qr.w, qf.w);
    EXPECT_FLOAT_EQ(qr.x, qf.x);
    EXPECT_FLOAT_EQ(qr.y, qf.y);
    EXPECT_FLOAT_EQ(qr.z, qf.z);
}

TEST_F(TestAhrsFilter, mahony_reset_then_rerun_matches_fresh_instance)
{
    math::Vector3<float> gyro{ { 0.2f }, { -0.1f }, { 0.3f } };
    math::Vector3<float> accel{ { 0.3f }, { 0.2f }, { g } };

    for (int i = 0; i < 100; ++i)
        mahony.UpdateImu(gyro, accel);

    mahony.Reset();

    filters::AhrsFilter<float, filters::AhrsMode::Mahony> fresh{ 0.5f, 0.01f };

    for (int i = 0; i < 50; ++i)
    {
        mahony.UpdateImu(gyro, accel);
        fresh.UpdateImu(gyro, accel);
    }

    math::Quaternion<float> qr{ mahony.Orientation() };
    math::Quaternion<float> qf{ fresh.Orientation() };
    EXPECT_FLOAT_EQ(qr.w, qf.w);
    EXPECT_FLOAT_EQ(qr.x, qf.x);
    EXPECT_FLOAT_EQ(qr.y, qf.y);
    EXPECT_FLOAT_EQ(qr.z, qf.z);
}

TEST_F(TestAhrsFilter, madgwick_single_step_gradient_correction)
{
    math::Vector3<float> gyro{ { 0.0f }, { 0.0f }, { 0.0f } };
    constexpr float rollRad{ 30.0f * std::numbers::pi_v<float> / 180.0f };
    math::Vector3<float> accel{ { 0.0f }, { g * std::sin(rollRad) }, { g * std::cos(rollRad) } };

    madgwick.UpdateImu(gyro, accel);

    math::Quaternion<float> orient{ madgwick.Orientation() };
    EXPECT_NEAR(orient.w, 0.99999950f, 1e-5f);
    EXPECT_NEAR(orient.x, 0.00100000f, 1e-5f);
    EXPECT_NEAR(orient.y, 0.0f, 1e-6f);
    EXPECT_NEAR(orient.z, 0.0f, 1e-6f);
}

TEST_F(TestAhrsFilter, mahony_single_step_proportional_correction)
{
    math::Vector3<float> gyro{ { 0.0f }, { 0.0f }, { 0.0f } };
    constexpr float rollRad{ 30.0f * std::numbers::pi_v<float> / 180.0f };
    math::Vector3<float> accel{ { 0.0f }, { g * std::sin(rollRad) }, { g * std::cos(rollRad) } };

    mahony.UpdateImu(gyro, accel);

    math::Quaternion<float> orient{ mahony.Orientation() };
    EXPECT_NEAR(orient.w, 0.99999922f, 1e-5f);
    EXPECT_NEAR(orient.x, 0.00125025f, 1e-5f);
    EXPECT_NEAR(orient.y, 0.0f, 1e-6f);
    EXPECT_NEAR(orient.z, 0.0f, 1e-6f);
}

TEST_F(TestAhrsFilter, madgwick_large_gyro_norm_preserved)
{
    math::Vector3<float> gyro{ { 20.0f }, { 15.0f }, { -10.0f } };
    math::Vector3<float> accel{ { 0.0f }, { 0.0f }, { g } };

    for (int i = 0; i < 200; ++i)
    {
        madgwick.UpdateImu(gyro, accel);
        float norm{ madgwick.Orientation().Norm() };
        EXPECT_NEAR(norm, 1.0f, 1e-4f);
    }
}

TEST_F(TestAhrsFilter, mahony_large_gyro_norm_preserved)
{
    math::Vector3<float> gyro{ { 20.0f }, { 15.0f }, { -10.0f } };
    math::Vector3<float> accel{ { 0.0f }, { 0.0f }, { g } };

    for (int i = 0; i < 200; ++i)
    {
        mahony.UpdateImu(gyro, accel);
        float norm{ mahony.Orientation().Norm() };
        EXPECT_NEAR(norm, 1.0f, 1e-4f);
    }
}

TEST_F(TestAhrsFilter, madgwick_pitch_recovery)
{
    constexpr float pitchDeg{ 20.0f };
    constexpr float pitchRad{ pitchDeg * std::numbers::pi_v<float> / 180.0f };

    math::Vector3<float> gyro{ { 0.0f }, { 0.0f }, { 0.0f } };
    math::Vector3<float> accel{ { -g * std::sin(pitchRad) }, { 0.0f }, { g * std::cos(pitchRad) } };

    for (int i = 0; i < 3000; ++i)
        madgwick.UpdateImu(gyro, accel);

    math::Vector3<float> euler{ madgwick.Euler() };
    float pitchResult{ euler.at(1, 0) * 180.0f / std::numbers::pi_v<float> };
    EXPECT_NEAR(pitchResult, pitchDeg, 1.0f);
    EXPECT_NEAR(euler.at(0, 0), 0.0f, 1e-2f);
}

TEST_F(TestAhrsFilter, mahony_pitch_recovery)
{
    constexpr float pitchDeg{ 20.0f };
    constexpr float pitchRad{ pitchDeg * std::numbers::pi_v<float> / 180.0f };

    math::Vector3<float> gyro{ { 0.0f }, { 0.0f }, { 0.0f } };
    math::Vector3<float> accel{ { -g * std::sin(pitchRad) }, { 0.0f }, { g * std::cos(pitchRad) } };

    for (int i = 0; i < 5000; ++i)
        mahony.UpdateImu(gyro, accel);

    math::Vector3<float> euler{ mahony.Euler() };
    float pitchResult{ euler.at(1, 0) * 180.0f / std::numbers::pi_v<float> };
    EXPECT_NEAR(pitchResult, pitchDeg, 1.0f);
    EXPECT_NEAR(euler.at(0, 0), 0.0f, 1e-2f);
}

TEST_F(TestAhrsFilter, mahony_marg_zero_accel_skips_correction)
{
    math::Vector3<float> gyro{ { 0.0f }, { 0.0f }, { 1.0f } };
    math::Vector3<float> accel{ { 0.0f }, { 0.0f }, { 0.0f } };
    math::Vector3<float> mag{ { 1.0f }, { 0.0f }, { 0.0f } };

    for (int i = 0; i < 100; ++i)
        mahony.UpdateMarg(gyro, accel, mag);

    math::Quaternion<float> orient{ mahony.Orientation() };
    EXPECT_NEAR(orient.Norm(), 1.0f, 1e-4f);
    EXPECT_FALSE(std::isnan(orient.w));
    EXPECT_FALSE(std::isnan(orient.x));
    EXPECT_FALSE(std::isnan(orient.y));
    EXPECT_FALSE(std::isnan(orient.z));
}

TEST_F(TestAhrsFilter, mahony_marg_zero_mag_falls_back_to_imu)
{
    constexpr float rollDeg{ 30.0f };
    constexpr float rollRad{ rollDeg * std::numbers::pi_v<float> / 180.0f };

    math::Vector3<float> gyro{ { 0.0f }, { 0.0f }, { 0.0f } };
    math::Vector3<float> accel{ { 0.0f }, { g * std::sin(rollRad) }, { g * std::cos(rollRad) } };
    math::Vector3<float> mag{ { 0.0f }, { 0.0f }, { 0.0f } };

    for (int i = 0; i < 5000; ++i)
        mahony.UpdateMarg(gyro, accel, mag);

    math::Vector3<float> euler{ mahony.Euler() };
    EXPECT_NEAR(euler.at(0, 0) * 180.0f / std::numbers::pi_v<float>, rollDeg, 1.0f);
}

TEST_F(TestAhrsFilter, madgwick_marg_pitch_recovery)
{
    constexpr float pitchDeg{ 20.0f };
    constexpr float pitchRad{ pitchDeg * std::numbers::pi_v<float> / 180.0f };

    math::Vector3<float> gyro{ { 0.0f }, { 0.0f }, { 0.0f } };
    math::Vector3<float> accel{ { -g * std::sin(pitchRad) }, { 0.0f }, { g * std::cos(pitchRad) } };
    math::Vector3<float> mag{ { 1.0f }, { 0.0f }, { 0.0f } };

    for (int i = 0; i < 3000; ++i)
        madgwick.UpdateMarg(gyro, accel, mag);

    math::Vector3<float> euler{ madgwick.Euler() };
    float pitchResult{ euler.at(1, 0) * 180.0f / std::numbers::pi_v<float> };
    EXPECT_NEAR(pitchResult, pitchDeg, 1.0f);
    EXPECT_NEAR(euler.at(0, 0), 0.0f, 1e-2f);
}

TEST_F(TestAhrsFilter, madgwick_no_nan_inf_at_extreme_accel)
{
    math::Vector3<float> gyro{ { 0.0f }, { 0.0f }, { 0.0f } };
    math::Vector3<float> accel{ { 1e-37f }, { 0.0f }, { 0.0f } };

    madgwick.UpdateImu(gyro, accel);

    math::Quaternion<float> orient{ madgwick.Orientation() };
    EXPECT_FALSE(std::isnan(orient.w));
    EXPECT_FALSE(std::isinf(orient.w));
    EXPECT_FALSE(std::isnan(orient.x));
    EXPECT_FALSE(std::isinf(orient.x));
}

TEST_F(TestAhrsFilter, mahony_no_nan_inf_at_extreme_accel)
{
    math::Vector3<float> gyro{ { 0.0f }, { 0.0f }, { 0.0f } };
    math::Vector3<float> accel{ { 1e-37f }, { 0.0f }, { 0.0f } };

    mahony.UpdateImu(gyro, accel);

    math::Quaternion<float> orient{ mahony.Orientation() };
    EXPECT_FALSE(std::isnan(orient.w));
    EXPECT_FALSE(std::isinf(orient.w));
    EXPECT_FALSE(std::isnan(orient.x));
    EXPECT_FALSE(std::isinf(orient.x));
}

namespace
{
    struct BodyPrediction
    {
        float x;
        float y;
        float z;
    };

    BodyPrediction RotateEarthToBody(const math::Quaternion<float>& q, float ex, float ez)
    {
        const float qw = q.w;
        const float qx = q.x;
        const float qy = q.y;
        const float qz = q.z;

        return BodyPrediction{
            ex * (1.0f - 2.0f * (qy * qy + qz * qz)) + ez * 2.0f * (qx * qz - qw * qy),
            ex * 2.0f * (qx * qy - qw * qz) + ez * 2.0f * (qw * qx + qy * qz),
            ex * 2.0f * (qw * qy + qx * qz) + ez * (1.0f - 2.0f * (qx * qx + qy * qy))
        };
    }

    class TestAhrsTiltedMarg : public ::testing::Test
    {
    protected:
        static constexpr float g{ 9.81f };

        filters::AhrsFilter<float, filters::AhrsMode::Madgwick> madgwick{ 0.5f, 0.01f };

        math::Quaternion<float> truth{ 0.8446f, 0.1913f, 0.4619f, 0.1802f };

        static constexpr float inclination{ 1.0472f };
        float fieldX{ std::cos(inclination) };
        float fieldZ{ std::sin(inclination) };

        void SetUp() override
        {
            truth.Normalize();
        }
    };
}

namespace
{
    float MagneticObjective(const math::Quaternion<float>& q, float mx, float my, float mz, float bx, float bz)
    {
        const float qw = q.w;
        const float qx = q.x;
        const float qy = q.y;
        const float qz = q.z;

        const float f4 = bx * (1.0f - 2.0f * (qy * qy + qz * qz)) + bz * 2.0f * (qx * qz - qw * qy) - mx;
        const float f5 = bx * 2.0f * (qx * qy - qw * qz) + bz * 2.0f * (qw * qx + qy * qz) - my;
        const float f6 = bx * 2.0f * (qw * qy + qx * qz) + bz * (1.0f - 2.0f * (qx * qx + qy * qy)) - mz;

        return 0.5f * (f4 * f4 + f5 * f5 + f6 * f6);
    }
}

TEST_F(TestAhrsTiltedMarg, magnetic_gradient_matches_finite_differences)
{
    using Filter = filters::AhrsFilter<float, filters::AhrsMode::Madgwick>;

    const std::array<math::Quaternion<float>, 3> orientations{
        math::Quaternion<float>{ 0.8446f, 0.1913f, 0.4619f, 0.1802f },
        math::Quaternion<float>{ 0.5963f, -0.4802f, 0.3521f, 0.5361f },
        math::Quaternion<float>{ 0.7071f, 0.0f, 0.7071f, 0.0f }
    };

    const float mx = 0.31f;
    const float my = -0.47f;
    const float mz = 0.82f;
    const float bx = 0.5f;
    const float bz = 0.87f;

    for (auto quat : orientations)
    {
        quat.Normalize();

        math::Vector3<float> mag{};
        mag.at(0, 0) = mx;
        mag.at(1, 0) = my;
        mag.at(2, 0) = mz;

        const auto analytic = Filter::GradientMag(quat, mag, bx, bz);

        const float step = 1e-3f;
        std::array<float, 4> numeric{};
        for (std::size_t i = 0; i < 4; ++i)
        {
            math::Quaternion<float> plus{ quat };
            math::Quaternion<float> minus{ quat };
            float* plusComponents[4]{ &plus.w, &plus.x, &plus.y, &plus.z };
            float* minusComponents[4]{ &minus.w, &minus.x, &minus.y, &minus.z };
            *plusComponents[i] += step;
            *minusComponents[i] -= step;

            numeric[i] = (MagneticObjective(plus, mx, my, mz, bx, bz) - MagneticObjective(minus, mx, my, mz, bx, bz)) / (2.0f * step);
        }

        float norm = std::sqrt(numeric[0] * numeric[0] + numeric[1] * numeric[1] + numeric[2] * numeric[2] + numeric[3] * numeric[3]);
        ASSERT_GT(norm, 1e-4f);

        EXPECT_NEAR(analytic.w, numeric[0] / norm, 1e-3f);
        EXPECT_NEAR(analytic.x, numeric[1] / norm, 1e-3f);
        EXPECT_NEAR(analytic.y, numeric[2] / norm, 1e-3f);
        EXPECT_NEAR(analytic.z, numeric[3] / norm, 1e-3f);
    }
}

TEST_F(TestAhrsTiltedMarg, converges_for_arbitrary_tilted_magnetic_field)
{
    const auto gravityBody = RotateEarthToBody(truth, 0.0f, 1.0f);
    const auto magBody = RotateEarthToBody(truth, fieldX, fieldZ);

    math::Vector3<float> gyro{};
    math::Vector3<float> accel{};
    accel.at(0, 0) = gravityBody.x * g;
    accel.at(1, 0) = gravityBody.y * g;
    accel.at(2, 0) = gravityBody.z * g;

    math::Vector3<float> mag{};
    mag.at(0, 0) = magBody.x;
    mag.at(1, 0) = magBody.y;
    mag.at(2, 0) = magBody.z;

    for (int i = 0; i < 20000; ++i)
        madgwick.UpdateMarg(gyro, accel, mag);

    const auto estimate = madgwick.Orientation();
    const auto predictedGravity = RotateEarthToBody(estimate, 0.0f, 1.0f);
    const auto predictedMag = RotateEarthToBody(estimate, fieldX, fieldZ);

    EXPECT_NEAR(predictedGravity.x, gravityBody.x, 2e-2f);
    EXPECT_NEAR(predictedGravity.y, gravityBody.y, 2e-2f);
    EXPECT_NEAR(predictedGravity.z, gravityBody.z, 2e-2f);

    EXPECT_NEAR(predictedMag.x, magBody.x, 2e-2f);
    EXPECT_NEAR(predictedMag.y, magBody.y, 2e-2f);
    EXPECT_NEAR(predictedMag.z, magBody.z, 2e-2f);
}
