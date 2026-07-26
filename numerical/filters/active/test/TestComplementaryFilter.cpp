#include "numerical/filters/active/ComplementaryFilter.hpp"
#include "numerical/math/Tolerance.hpp"
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    class TestComplementaryFilter
        : public ::testing::Test
    {
    public:
        filters::ComplementaryFilter<float> filter{ 0.98f, 0.01f };
    };
}

TEST_F(TestComplementaryFilter, static_measurement_converges)
{
    float output{};
    for (int i = 0; i < 500; ++i)
        output = filter.Update(0.0f, 0.2f);

    EXPECT_NEAR(output, 0.2f, math::Tolerance<float>());
}

TEST_F(TestComplementaryFilter, pure_rotation_follows_gyro)
{
    constexpr float w{ 1.0f };
    constexpr float Ts{ 0.01f };
    constexpr int n{ 5 };

    float output{};
    for (int i = 0; i < n; ++i)
        output = filter.Update(w, 0.0f);

    EXPECT_GT(output, 0.0f);
    EXPECT_LT(output, w * Ts * static_cast<float>(n) + math::Tolerance<float>());
}

TEST_F(TestComplementaryFilter, rejects_slow_sensor_noise)
{
    constexpr int N{ 100 };
    float noisy[N]{};
    float filtered[N]{};

    for (int i = 0; i < N; ++i)
    {
        noisy[i] = (i % 2 == 0) ? 0.1f : -0.1f;
        filtered[i] = filter.Update(0.0f, noisy[i]);
    }

    float varNoisy{};
    float varFiltered{};
    for (int i = 0; i < N; ++i)
    {
        varNoisy += noisy[i] * noisy[i];
        varFiltered += filtered[i] * filtered[i];
    }

    EXPECT_LT(varFiltered, varNoisy);
}

TEST_F(TestComplementaryFilter, rejects_gyro_bias_drift)
{
    constexpr float bias{ 0.01f };
    constexpr int N{ 1000 };

    float output{};
    for (int i = 0; i < N; ++i)
        output = filter.Update(bias, 0.0f);

    EXPECT_LT(std::abs(output), 1.0f);
}

TEST_F(TestComplementaryFilter, alpha_from_tau_matches_crossover)
{
    constexpr float tau{ 1.0f };
    constexpr float Ts{ 0.01f };

    const float a{ filters::ComplementaryFilter<float>::AlphaFromTau(tau, Ts) };

    EXPECT_NEAR(a, tau / (tau + Ts), math::Tolerance<float>());
}

TEST_F(TestComplementaryFilter, heading_blend_wraps_shortest_arc)
{
    constexpr float pi{ std::numbers::pi_v<float> };
    filters::ComplementaryFilter<float> headingFilter{ 0.98f, 0.01f, pi - 0.05f, true };

    float output{};
    for (int i = 0; i < 10; ++i)
        output = headingFilter.Update(0.0f, -pi + 0.05f);

    EXPECT_LE(std::abs(output), pi + math::Tolerance<float>());
}

TEST_F(TestComplementaryFilter, reset_sets_state)
{
    for (int i = 0; i < 10; ++i)
        filter.Update(1.0f, 0.5f);

    filter.Reset(0.1f);

    const float output{ filter.Update(0.0f, 0.0f) };
    EXPECT_NEAR(output, 0.98f * 0.1f, math::Tolerance<float>());
}

TEST_F(TestComplementaryFilter, alpha_zero_is_pure_measurement)
{
    filters::ComplementaryFilter<float> pureAccel{ 0.0f, 0.01f };

    const float output{ pureAccel.Update(5.0f, 0.3f) };

    EXPECT_NEAR(output, 0.3f, math::Tolerance<float>());
}

TEST_F(TestComplementaryFilter, alpha_one_is_pure_integration)
{
    filters::ComplementaryFilter<float> pureGyro{ 1.0f, 0.01f };

    const float output{ pureGyro.Update(10.0f, 999.0f) };

    EXPECT_NEAR(output, 10.0f * 0.01f, math::Tolerance<float>());
}
