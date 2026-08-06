#include "numerical/filters/passive/CicFilter.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gmock/gmock.h"
#include <array>
#include <cmath>
#include <numbers>

namespace
{
    class TestCicDecimator
        : public ::testing::Test
    {
    public:
        filters::passive::CicDecimator<float, 2, 4, 1> cic{};
    };
}

TEST_F(TestCicDecimator, gain_matches_analytic_formula)
{
    constexpr float gain = filters::passive::CicDecimator<float, 2, 4, 1>::Gain();
    EXPECT_FLOAT_EQ(gain, 16.0f);
}

TEST_F(TestCicDecimator, gain_single_stage)
{
    constexpr float gain = filters::passive::CicDecimator<float, 1, 8, 1>::Gain();
    EXPECT_FLOAT_EQ(gain, 8.0f);
}

TEST_F(TestCicDecimator, gain_triple_stage_differential_delay_two)
{
    constexpr float gain = filters::passive::CicDecimator<float, 3, 2, 2>::Gain();
    EXPECT_FLOAT_EQ(gain, 64.0f);
}

TEST_F(TestCicDecimator, emits_one_output_per_decimation_ratio)
{
    int count{ 0 };
    for (int i = 0; i < 8; ++i)
    {
        auto result = cic.Filter(1.0f);
        if (result.valid)
            ++count;
    }
    EXPECT_EQ(count, 2);
}

TEST_F(TestCicDecimator, invalid_sample_has_zero_value)
{
    auto result = cic.Filter(1.0f);
    EXPECT_FALSE(result.valid);
    EXPECT_FLOAT_EQ(result.value, 0.0f);
}

TEST_F(TestCicDecimator, dc_gain_normalised_to_unity)
{
    constexpr float c{ 0.7f };
    filters::passive::CicSample<float> last{};
    for (int i = 0; i < 24; ++i)
    {
        auto r = cic.Filter(c);
        if (r.valid)
            last = r;
    }
    EXPECT_NEAR(last.value, c, math::Tolerance<float>());
}

TEST_F(TestCicDecimator, impulse_first_output_matches_doc_walkthrough)
{
    constexpr float tol{ 1e-6f };
    filters::passive::CicSample<float> first{};
    for (int i = 0; i < 4; ++i)
    {
        float inp = (i == 0) ? 1.0f : 0.0f;
        auto r = cic.Filter(inp);
        if (r.valid)
            first = r;
    }
    EXPECT_NEAR(first.value, 0.25f, tol);
}

TEST_F(TestCicDecimator, impulse_second_output_is_zero)
{
    constexpr float tol{ 1e-6f };
    int count{ 0 };
    filters::passive::CicSample<float> second{};
    for (int i = 0; i < 8; ++i)
    {
        float inp = (i == 0) ? 1.0f : 0.0f;
        auto r = cic.Filter(inp);
        if (r.valid)
        {
            ++count;
            if (count == 2)
                second = r;
        }
    }
    EXPECT_NEAR(second.value, 0.0f, tol);
}

TEST_F(TestCicDecimator, step_response_settles_to_input_after_two_decimated_samples)
{
    constexpr float tol{ 1e-6f };
    filters::passive::CicSample<float> last{};
    for (int i = 0; i < 12; ++i)
    {
        auto r = cic.Filter(1.0f);
        if (r.valid)
            last = r;
    }
    EXPECT_TRUE(last.valid);
    EXPECT_NEAR(last.value, 1.0f, tol);
}

TEST_F(TestCicDecimator, stopband_null_at_integer_multiple_of_decimation_rate)
{
    constexpr float tol{ 1e-5f };
    constexpr int nSamples{ 64 };
    std::array<float, nSamples> outputs{};
    int outCount{ 0 };

    for (int n = 0; n < nSamples; ++n)
    {
        float x = std::sin(2.0f * std::numbers::pi_v<float> * 0.25f * static_cast<float>(n));
        auto r = cic.Filter(x);
        if (r.valid && outCount < nSamples)
            outputs[outCount++] = r.value;
    }

    float rms{ 0.0f };
    int skipTransient{ 4 };
    int count{ outCount - skipTransient };
    for (int i = skipTransient; i < outCount; ++i)
        rms += outputs[i] * outputs[i];
    rms = std::sqrt(rms / static_cast<float>(count));

    EXPECT_NEAR(rms, 0.0f, tol);
}

TEST_F(TestCicDecimator, zero_input_gives_zero_output)
{
    constexpr float tol{ 1e-6f };
    for (int i = 0; i < 8; ++i)
    {
        auto result = cic.Filter(0.0f);
        if (result.valid)
            EXPECT_NEAR(result.value, 0.0f, tol);
    }
}

TEST_F(TestCicDecimator, reset_restores_initial_state)
{
    constexpr float tol{ 1e-6f };
    for (int i = 0; i < 4; ++i)
        cic.Filter(1.0f);
    cic.Reset();

    filters::passive::CicDecimator<float, 2, 4, 1> fresh{};
    filters::passive::CicSample<float> afterReset{};
    filters::passive::CicSample<float> fromFresh{};

    for (int i = 0; i < 4; ++i)
    {
        auto r1 = cic.Filter(1.0f);
        auto r2 = fresh.Filter(1.0f);
        if (r1.valid)
            afterReset = r1;
        if (r2.valid)
            fromFresh = r2;
    }
    EXPECT_NEAR(afterReset.value, fromFresh.value, tol);
}

TEST_F(TestCicDecimator, determinism_same_input_same_output)
{
    constexpr std::array<float, 8> input{ 0.1f, 0.5f, -0.3f, 0.8f, 0.2f, -0.6f, 0.4f, 0.9f };

    filters::passive::CicDecimator<float, 2, 4, 1> cic1{};
    filters::passive::CicDecimator<float, 2, 4, 1> cic2{};

    std::array<float, 2> out1{};
    std::array<float, 2> out2{};
    int idx1{ 0 };
    int idx2{ 0 };

    for (std::size_t i = 0; i < input.size(); ++i)
    {
        auto r1 = cic1.Filter(input[i]);
        auto r2 = cic2.Filter(input[i]);
        if (r1.valid && idx1 < 2)
            out1[idx1++] = r1.value;
        if (r2.valid && idx2 < 2)
            out2[idx2++] = r2.value;
    }

    EXPECT_FLOAT_EQ(out1[0], out2[0]);
    EXPECT_FLOAT_EQ(out1[1], out2[1]);
}

TEST_F(TestCicDecimator, two_instances_do_not_interfere)
{
    constexpr float tol{ 1e-6f };
    filters::passive::CicDecimator<float, 2, 4, 1> cicA{};
    filters::passive::CicDecimator<float, 2, 4, 1> cicB{};

    for (int i = 0; i < 24; ++i)
        cicA.Filter(1.0f);

    filters::passive::CicSample<float> lastB{};
    for (int i = 0; i < 4; ++i)
    {
        auto r = cicB.Filter(0.0f);
        if (r.valid)
            lastB = r;
    }
    EXPECT_NEAR(lastB.value, 0.0f, tol);
}

TEST_F(TestCicDecimator, m2_comb_delay_spreads_impulse_response)
{
    constexpr float tol{ 1e-6f };
    filters::passive::CicDecimator<float, 1, 4, 2> cicM2{};

    std::array<float, 4> outputs{};
    int outCount{ 0 };

    for (int i = 0; i < 16 && outCount < 4; ++i)
    {
        float inp = (i == 0) ? 1.0f : 0.0f;
        auto r = cicM2.Filter(inp);
        if (r.valid)
            outputs[outCount++] = r.value;
    }

    EXPECT_NEAR(outputs[0], 0.125f, tol);
    EXPECT_NEAR(outputs[1], 0.125f, tol);
    EXPECT_NEAR(outputs[2], 0.0f, tol);
    EXPECT_NEAR(outputs[3], 0.0f, tol);
}

TEST_F(TestCicDecimator, long_run_bounded_output_no_nan)
{
    constexpr int nSamples{ 1000 };
    float last{ 0.0f };
    for (int i = 0; i < nSamples; ++i)
    {
        auto r = cic.Filter(1.0f);
        if (r.valid)
            last = r.value;
    }
    EXPECT_FALSE(std::isnan(last));
    EXPECT_FALSE(std::isinf(last));
    EXPECT_NEAR(last, 1.0f, math::Tolerance<float>());
}
