#include "numerical/controllers/implementations/LeadLagCompensator.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gtest/gtest.h"

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
    EXPECT_NEAR(steadyState, expected, 1e-3f);
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
    EXPECT_NEAR(tail, 0.0f, 1e-3f);
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

TEST_F(TestLeadLagCompensator, step_reaches_expected_steady_state)
{
    for (int i = 0; i < 5000; ++i)
        comp.Compute(1.0f);

    float finalVal{ comp.Compute(1.0f) };
    float expected{ params.gain * params.zero / params.pole };
    EXPECT_NEAR(finalVal, expected, 1e-3f);
}

TEST_F(TestLeadLagCompensator, unity_when_zero_equals_pole)
{
    controllers::LeadLagParameters<float> unityParams{ 2.0f, 5.0f, 5.0f, 0.01f };
    controllers::LeadLagCompensator<float> unityComp{ unityParams };

    for (int i = 0; i < 2000; ++i)
        unityComp.Compute(1.0f);

    float steadyState{ unityComp.Compute(1.0f) };
    EXPECT_NEAR(steadyState, unityParams.gain, 1e-3f);
}
