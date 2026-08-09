#include "numerical/controllers/implementations/LeadLagCompensator.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gtest/gtest.h"
#include <cmath>

namespace
{
    class TestLeadLagCompensator
        : public ::testing::Test
    {
    public:
        controllers::LeadLagParameters<float> params{ 1.0f, 1.0f, 10.0f, 0.01f };
        controllers::LeadLagCompensator<float> comp{ params };
    };
}

TEST_F(TestLeadLagCompensator, dc_gain_matches_continuous)
{
    for (int i = 0; i < 2000; ++i)
        comp.Compute(1.0f);

    float steadyState{ comp.Compute(1.0f) };
    float expected{ params.gain * params.zero / params.pole };
    EXPECT_NEAR(steadyState, expected, math::Tolerance<float>());
}

TEST_F(TestLeadLagCompensator, lead_produces_initial_overshoot)
{
    float firstOut{ comp.Compute(1.0f) };

    for (int i = 0; i < 2000; ++i)
        comp.Compute(1.0f);
    float steadyState{ comp.Compute(1.0f) };

    EXPECT_GT(firstOut, steadyState);
}

TEST_F(TestLeadLagCompensator, lag_has_no_derivative_kick)
{
    controllers::LeadLagParameters<float> lagParams{ 1.0f, 10.0f, 1.0f, 0.01f };
    controllers::LeadLagCompensator<float> lagComp{ lagParams };

    float prev{ lagComp.Compute(1.0f) };
    bool monotone{ true };
    for (int i = 0; i < 100; ++i)
    {
        float cur{ lagComp.Compute(1.0f) };
        if (cur < prev)
        {
            monotone = false;
            break;
        }
        prev = cur;
    }
    EXPECT_TRUE(monotone);
}

TEST_F(TestLeadLagCompensator, impulse_response_is_stable)
{
    comp.Compute(1.0f);
    for (int i = 0; i < 1000; ++i)
        comp.Compute(0.0f);

    float tail{ comp.Compute(0.0f) };
    EXPECT_NEAR(tail, 0.0f, math::Tolerance<float>());
}

TEST_F(TestLeadLagCompensator, coefficients_match_tustin_design)
{
    float c{ 2.0f / params.sampleTime };
    float d0{ c + params.pole };
    float expectedB0{ params.gain * (c + params.zero) / d0 };
    float expectedB1{ params.gain * (params.zero - c) / d0 };
    float expectedA1{ (params.pole - c) / d0 };

    float y0{ comp.Compute(1.0f) };
    EXPECT_NEAR(y0, expectedB0, 1e-5f);

    float y1{ comp.Compute(0.0f) };
    EXPECT_NEAR(y1, expectedB1 - expectedA1 * y0, 1e-5f);
}

TEST_F(TestLeadLagCompensator, reset_clears_history)
{
    for (int i = 0; i < 10; ++i)
        comp.Compute(1.0f);

    comp.Reset(0.0f);

    float c{ 2.0f / params.sampleTime };
    float d0{ c + params.pole };
    float b0{ params.gain * (c + params.zero) / d0 };

    float nextInput{ 0.5f };
    float out{ comp.Compute(nextInput) };
    EXPECT_NEAR(out, b0 * nextInput, 1e-5f);
}

TEST_F(TestLeadLagCompensator, unity_when_zero_equals_pole)
{
    controllers::LeadLagParameters<float> unityParams{ 2.0f, 5.0f, 5.0f, 0.01f };
    controllers::LeadLagCompensator<float> unityComp{ unityParams };

    for (int i = 0; i < 2000; ++i)
        unityComp.Compute(1.0f);

    float steadyState{ unityComp.Compute(1.0f) };
    EXPECT_NEAR(steadyState, unityParams.gain, math::Tolerance<float>());
}

TEST_F(TestLeadLagCompensator, determinism_same_input_same_output)
{
    std::array<float, 8> sequence{ 1.0f, 0.5f, -0.3f, 0.8f, 0.0f, 1.2f, -1.0f, 0.4f };
    std::array<float, 8> out1{};
    std::array<float, 8> out2{};

    for (int i = 0; i < 8; ++i)
        out1[i] = comp.Compute(sequence[i]);

    comp.Reset(0.0f);

    for (int i = 0; i < 8; ++i)
        out2[i] = comp.Compute(sequence[i]);

    for (int i = 0; i < 8; ++i)
        EXPECT_FLOAT_EQ(out1[i], out2[i]);
}

TEST_F(TestLeadLagCompensator, nyquist_frequency_gain_matches_analytic)
{
    float analyticMag{ params.gain };

    std::array<float, 512> signs{};
    for (int i = 0; i < 512; ++i)
        signs[i] = (i % 2 == 0) ? 1.0f : -1.0f;

    for (int i = 0; i < 400; ++i)
        comp.Compute(signs[i]);

    float sumSq{ 0.0f };
    for (int i = 400; i < 512; ++i)
    {
        float y{ comp.Compute(signs[i]) };
        sumSq += y * y;
    }

    float rms{ std::sqrt(sumSq / 112.0f) };
    EXPECT_NEAR(rms, analyticMag, math::Tolerance<float>());
}

TEST_F(TestLeadLagCompensator, zero_input_zero_output)
{
    for (int i = 0; i < 50; ++i)
    {
        float out{ comp.Compute(0.0f) };
        EXPECT_FLOAT_EQ(out, 0.0f);
    }
}

TEST_F(TestLeadLagCompensator, negative_gain_dc_and_stability)
{
    controllers::LeadLagParameters<float> negParams{ -2.0f, 1.0f, 10.0f, 0.01f };
    controllers::LeadLagCompensator<float> negComp{ negParams };

    negComp.Compute(1.0f);
    for (int i = 0; i < 1000; ++i)
        negComp.Compute(0.0f);

    float tail{ negComp.Compute(0.0f) };
    EXPECT_NEAR(tail, 0.0f, math::Tolerance<float>());

    negComp.Reset(0.0f);
    for (int i = 0; i < 2000; ++i)
        negComp.Compute(1.0f);

    float dcSteady{ negComp.Compute(1.0f) };
    float expected{ negParams.gain * negParams.zero / negParams.pole };
    EXPECT_NEAR(dcSteady, expected, math::Tolerance<float>());
}
