#include "numerical/filters/active/ComplementaryFilter.hpp"
#include "numerical/math/Tolerance.hpp"
#include <array>
#include <cmath>
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

TEST_F(TestComplementaryFilter, single_step_matches_closed_form)
{
    constexpr float rate{ 0.1745f };
    constexpr float accel{ 0.01f };
    constexpr float alpha{ 0.98f };
    constexpr float Ts{ 0.01f };

    const float predicted{ 0.0f + rate * Ts };
    const float expected{ alpha * predicted + (1.0f - alpha) * accel };

    const float output{ filter.Update(rate, accel) };

    EXPECT_NEAR(output, expected, math::Tolerance<float>());
}

TEST_F(TestComplementaryFilter, gyro_only_integrates_exactly)
{
    constexpr float rate{ 1.0f };
    constexpr float Ts{ 0.01f };
    constexpr float alpha{ 1.0f };

    filters::ComplementaryFilter<float> gyroOnly{ alpha, Ts };

    constexpr int n{ 5 };
    float output{};
    for (int i = 0; i < n; ++i)
        output = gyroOnly.Update(rate, 0.0f);

    EXPECT_NEAR(output, rate * Ts * static_cast<float>(n), math::Tolerance<float>());
}

TEST_F(TestComplementaryFilter, accel_only_tracks_measurement)
{
    constexpr float rate{ 99.0f };
    constexpr float accel{ 0.3f };
    filters::ComplementaryFilter<float> pureAccel{ 0.0f, 0.01f };

    const float output{ pureAccel.Update(rate, accel) };

    EXPECT_NEAR(output, accel, math::Tolerance<float>());
}

TEST_F(TestComplementaryFilter, static_measurement_convergence_rate)
{
    constexpr float alpha{ 0.98f };
    constexpr float target{ 0.2f };
    constexpr int n{ 50 };

    float output{};
    for (int i = 0; i < n; ++i)
        output = filter.Update(0.0f, target);

    const float expectedError{ std::pow(alpha, static_cast<float>(n)) * (0.0f - target) };
    const float expected{ target + expectedError };

    EXPECT_NEAR(output, expected, math::Tolerance<float>());
}

TEST_F(TestComplementaryFilter, gyro_bias_steady_state_bound)
{
    constexpr float bias{ 0.01f };
    constexpr float alpha{ 0.98f };
    constexpr float Ts{ 0.01f };
    constexpr float bound{ bias * Ts * alpha / (1.0f - alpha) };

    float output{};
    for (int i = 0; i < 2000; ++i)
        output = filter.Update(bias, 0.0f);

    EXPECT_NEAR(output, bound, 5.0f * math::Tolerance<float>());
}

TEST_F(TestComplementaryFilter, complementary_sum_unity)
{
    constexpr float rate{ 0.5f };
    constexpr float accel{ 0.7f };
    constexpr float alpha{ 0.98f };
    constexpr float Ts{ 0.01f };

    filters::ComplementaryFilter<float> gyroSide{ 1.0f, Ts };
    filters::ComplementaryFilter<float> accelSide{ 0.0f, Ts };

    const float hp{ gyroSide.Update(rate, accel) };
    const float lp{ accelSide.Update(rate, accel) };

    const float reference{ rate * Ts + accel };
    EXPECT_NEAR(hp + lp, reference, math::Tolerance<float>());
}

TEST_F(TestComplementaryFilter, alpha_from_tau_formula)
{
    constexpr float tau{ 1.0f };
    constexpr float Ts{ 0.01f };

    const float a{ filters::ComplementaryFilter<float>::AlphaFromTau(tau, Ts) };

    EXPECT_NEAR(a, tau / (tau + Ts), math::Tolerance<float>());
}

TEST_F(TestComplementaryFilter, reset_restores_initial_state)
{
    for (int i = 0; i < 20; ++i)
        filter.Update(1.0f, 0.5f);

    filter.Reset(0.1f);

    filters::ComplementaryFilter<float> freshAt01{ 0.98f, 0.01f, 0.1f };
    const float fromReset{ filter.Update(0.3f, 0.4f) };
    const float fromFresh{ freshAt01.Update(0.3f, 0.4f) };

    EXPECT_FLOAT_EQ(fromReset, fromFresh);
}

TEST_F(TestComplementaryFilter, determinism_same_inputs_same_output)
{
    filters::ComplementaryFilter<float> a{ 0.98f, 0.01f };
    filters::ComplementaryFilter<float> b{ 0.98f, 0.01f };

    constexpr std::array<float, 4> rates{ 0.1f, -0.2f, 0.05f, 1.0f };
    constexpr std::array<float, 4> accels{ 0.01f, 0.03f, -0.01f, 0.5f };

    for (int i = 0; i < 4; ++i)
    {
        const float outA{ a.Update(rates[i], accels[i]) };
        const float outB{ b.Update(rates[i], accels[i]) };
        EXPECT_FLOAT_EQ(outA, outB);
    }
}

TEST_F(TestComplementaryFilter, two_instances_do_not_interfere)
{
    filters::ComplementaryFilter<float> first{ 0.98f, 0.01f };
    filters::ComplementaryFilter<float> second{ 0.98f, 0.01f };

    for (int i = 0; i < 10; ++i)
        first.Update(5.0f, 1.0f);

    filters::ComplementaryFilter<float> reference{ 0.98f, 0.01f };
    const float refOut{ reference.Update(0.3f, 0.2f) };
    const float secondOut{ second.Update(0.3f, 0.2f) };

    EXPECT_FLOAT_EQ(secondOut, refOut);
}

TEST_F(TestComplementaryFilter, set_alpha_changes_blend_weight)
{
    filter.SetAlpha(0.0f);

    constexpr float accel{ 0.5f };
    const float output{ filter.Update(99.0f, accel) };

    EXPECT_NEAR(output, accel, math::Tolerance<float>());
}

TEST_F(TestComplementaryFilter, heading_wrap_shortest_arc_reference)
{
    constexpr float pi{ std::numbers::pi_v<float> };
    constexpr float initial{ pi - 0.05f };
    constexpr float target{ -pi + 0.05f };
    constexpr float alpha{ 0.98f };

    filters::ComplementaryFilter<float> headingFilter{ alpha, 0.01f, initial, true };

    const float delta{ target - initial };
    const float wrappedDelta{ delta + 2.0f * pi };
    const float expected{ initial + (1.0f - alpha) * wrappedDelta };

    const float output{ headingFilter.Update(0.0f, target) };

    EXPECT_NEAR(output, expected, math::Tolerance<float>());
}

TEST_F(TestComplementaryFilter, heading_output_stays_within_pi)
{
    constexpr float pi{ std::numbers::pi_v<float> };
    filters::ComplementaryFilter<float> headingFilter{ 0.98f, 0.01f, pi - 0.01f, true };

    float output{};
    for (int i = 0; i < 20; ++i)
        output = headingFilter.Update(0.1f, -pi + 0.01f);

    EXPECT_LE(std::abs(output), pi + math::Tolerance<float>());
}

TEST_F(TestComplementaryFilter, no_nan_inf_on_zero_inputs)
{
    const float output{ filter.Update(0.0f, 0.0f) };

    EXPECT_FALSE(std::isnan(output));
    EXPECT_FALSE(std::isinf(output));
    EXPECT_FLOAT_EQ(output, 0.0f);
}

TEST_F(TestComplementaryFilter, no_nan_inf_on_large_rate)
{
    constexpr float large{ 1.0e6f };
    const float output{ filter.Update(large, 0.0f) };

    EXPECT_FALSE(std::isnan(output));
    EXPECT_FALSE(std::isinf(output));
}
