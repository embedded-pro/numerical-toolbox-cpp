#include "numerical/math/StepResponseMetrics.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gmock/gmock.h"
#include <cmath>

namespace
{
    static constexpr std::size_t N = 64;
    using Vec = math::Vector<float, N>;

    class TestStepResponseMetrics : public ::testing::Test
    {
    protected:
        Vec MakeRampPlateau(float steady, std::size_t rampEnd) const
        {
            Vec v;
            for (std::size_t i = 0; i < N; ++i)
            {
                if (i <= rampEnd)
                    v.at(i, 0) = steady * static_cast<float>(i) / static_cast<float>(rampEnd);
                else
                    v.at(i, 0) = steady;
            }
            return v;
        }

        Vec MakeUnderdamped(float steady, float overshootFrac, std::size_t peakIdx) const
        {
            Vec v;
            const float peak{ steady * (1.0f + overshootFrac) };
            for (std::size_t i = 0; i < N; ++i)
            {
                const float t{ static_cast<float>(i) / static_cast<float>(peakIdx) };
                const float envelope{ 1.0f - std::exp(-2.0f * t) };
                const float osc{ overshootFrac * std::exp(-2.0f * t) * std::cos(std::numbers::pi_v<float> * t) }; // NOLINT
                v.at(i, 0) = steady * (envelope + osc);
            }
            (void)peak;
            return v;
        }
    };
}

TEST_F(TestStepResponseMetrics, rise_time_ramp_plateau_in_samples)
{
    const float steady{ 1.0f };
    const std::size_t rampEnd{ 20 };
    Vec v{ MakeRampPlateau(steady, rampEnd) };

    const float result{ math::RiseTime(v, steady) };

    EXPECT_NEAR(result, 16.0f, 2.0f);
}

TEST_F(TestStepResponseMetrics, rise_time_with_dt_scales_to_real_time)
{
    const float steady{ 1.0f };
    Vec v{ MakeRampPlateau(steady, 20) };
    const float dt{ 0.01f };

    const float sampleResult{ math::RiseTime(v, steady) };
    const float timeResult{ math::RiseTime(v, steady, dt) };

    EXPECT_NEAR(timeResult, sampleResult * dt, math::Tolerance<float>());
}

TEST_F(TestStepResponseMetrics, settling_time_at_steady_state_returns_zero)
{
    Vec v;
    for (std::size_t i = 0; i < N; ++i)
        v.at(i, 0) = 1.0f;

    const float result{ math::SettlingTime(v, 1.0f) };

    EXPECT_NEAR(result, 0.0f, math::Tolerance<float>());
}

TEST_F(TestStepResponseMetrics, settling_time_ramp_plateau_within_band)
{
    const float steady{ 1.0f };
    Vec v{ MakeRampPlateau(steady, 20) };

    const float result{ math::SettlingTime(v, steady, 0.02f) };

    EXPECT_GT(result, 0.0f);
    EXPECT_LT(result, static_cast<float>(N));
}

TEST_F(TestStepResponseMetrics, percent_overshoot_no_overshoot_returns_non_positive)
{
    Vec v{ MakeRampPlateau(1.0f, 30) };

    const float result{ math::PercentOvershoot(v, 1.0f) };

    EXPECT_LE(result, 0.0f);
}

TEST_F(TestStepResponseMetrics, percent_overshoot_known_peak)
{
    Vec v;
    for (std::size_t i = 0; i < N; ++i)
        v.at(i, 0) = 1.0f;
    v.at(10, 0) = 1.2f;

    const float result{ math::PercentOvershoot(v, 1.0f) };

    EXPECT_NEAR(result, 20.0f, 1e-3f);
}

TEST_F(TestStepResponseMetrics, percent_overshoot_zero_steady_returns_zero)
{
    Vec v{ MakeRampPlateau(1.0f, 20) };

    const float result{ math::PercentOvershoot(v, 0.0f) };

    EXPECT_NEAR(result, 0.0f, math::Tolerance<float>());
}

TEST_F(TestStepResponseMetrics, peak_time_finds_maximum_index)
{
    Vec v;
    for (std::size_t i = 0; i < N; ++i)
        v.at(i, 0) = 1.0f;
    v.at(15, 0) = 1.5f;

    const float result{ math::PeakTime(v) };

    EXPECT_NEAR(result, 15.0f, math::Tolerance<float>());
}

TEST_F(TestStepResponseMetrics, peak_time_with_dt_scales_to_real_time)
{
    Vec v;
    for (std::size_t i = 0; i < N; ++i)
        v.at(i, 0) = 1.0f;
    v.at(15, 0) = 1.5f;
    const float dt{ 0.005f };

    const float result{ math::PeakTime(v, dt) };

    EXPECT_NEAR(result, 15.0f * dt, math::Tolerance<float>());
}

TEST_F(TestStepResponseMetrics, steady_state_error_zero_for_perfect_step)
{
    Vec v;
    for (std::size_t i = 0; i < N; ++i)
        v.at(i, 0) = 1.0f;

    const float result{ math::SteadyStateError(v, 1.0f) };

    EXPECT_NEAR(result, 0.0f, 1e-4f);
}

TEST_F(TestStepResponseMetrics, steady_state_error_known_offset)
{
    Vec v;
    for (std::size_t i = 0; i < N; ++i)
        v.at(i, 0) = 0.9f;

    const float result{ math::SteadyStateError(v, 1.0f) };

    EXPECT_NEAR(result, 0.1f, 1e-4f);
}

TEST_F(TestStepResponseMetrics, rise_time_never_reaches_threshold_returns_max)
{
    Vec v;
    for (std::size_t i = 0; i < N; ++i)
        v.at(i, 0) = 0.05f;

    const float result{ math::RiseTime(v, 1.0f) };

    EXPECT_NEAR(result, static_cast<float>(N - 1), math::Tolerance<float>());
}

TEST_F(TestStepResponseMetrics, rise_time_reaches_lo_but_not_hi_returns_max)
{
    Vec v;
    for (std::size_t i = 0; i < N; ++i)
        v.at(i, 0) = (i < 10) ? 0.0f : 0.5f;

    const float result{ math::RiseTime(v, 1.0f) };

    EXPECT_NEAR(result, static_cast<float>(N - 1), math::Tolerance<float>());
}

TEST_F(TestStepResponseMetrics, settling_time_with_dt_scales_result)
{
    Vec v{ MakeRampPlateau(1.0f, 20) };
    const float dt{ 0.01f };

    const float sampleResult{ math::SettlingTime(v, 1.0f, 0.02f) };
    const float timeResult{ math::SettlingTime(v, 1.0f, 0.02f, dt) };

    EXPECT_NEAR(timeResult, sampleResult * dt, math::Tolerance<float>());
}

TEST_F(TestStepResponseMetrics, settling_time_always_outside_band_returns_full_duration)
{
    Vec v;
    for (std::size_t i = 0; i < N; ++i)
        v.at(i, 0) = 0.0f;

    const float result{ math::SettlingTime(v, 1.0f, 0.02f) };

    EXPECT_NEAR(result, static_cast<float>(N), math::Tolerance<float>());
}

TEST_F(TestStepResponseMetrics, peak_time_at_index_zero_for_monotone_decreasing)
{
    Vec v;
    for (std::size_t i = 0; i < N; ++i)
        v.at(i, 0) = static_cast<float>(N - i);

    const float result{ math::PeakTime(v) };

    EXPECT_NEAR(result, 0.0f, math::Tolerance<float>());
}

TEST_F(TestStepResponseMetrics, steady_state_error_custom_tail_size)
{
    Vec v;
    for (std::size_t i = 0; i < N; ++i)
        v.at(i, 0) = (i < N / 2) ? 0.0f : 0.8f;

    const float result{ math::SteadyStateError<float, N, 8>(v, 1.0f) };

    EXPECT_NEAR(result, 0.2f, 1e-4f);
}
