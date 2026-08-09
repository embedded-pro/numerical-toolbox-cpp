#include "numerical/filters/passive/Iir.hpp"
#include "numerical/math/RecursiveBuffer.hpp"
#include "numerical/math/Tolerance.hpp"
#include <gtest/gtest.h>

namespace
{
    class TestIir : public ::testing::Test
    {
    protected:
        static constexpr std::size_t Order = 3;

        math::RecursiveBuffer<float, Order> MakeCoeffs(float c0, float c1, float c2 = 0.0f)
        {
            math::RecursiveBuffer<float, Order> buf{};
            buf = { c0, c1, c2 };
            return buf;
        }
    };
}

TEST_F(TestIir, feedforward_and_feedback_step_sequence)
{
    auto b = MakeCoeffs(0.1f, 0.0f);
    auto a = MakeCoeffs(0.5f, 0.4f);
    filters::passive::Iir<float, Order, Order> filter{ b, a };

    EXPECT_NEAR(filter.Filter(0.5f), 0.05f, math::Tolerance<float>());
    EXPECT_NEAR(filter.Filter(0.5f), 0.075f, math::Tolerance<float>());
    EXPECT_NEAR(filter.Filter(0.5f), 0.1075f, math::Tolerance<float>());
}

TEST_F(TestIir, feedforward_with_negative_feedback_step_sequence)
{
    auto b = MakeCoeffs(0.1f, 0.2f);
    auto a = MakeCoeffs(0.5f, -0.1f);
    filters::passive::Iir<float, Order, Order> filter{ b, a };

    EXPECT_NEAR(filter.Filter(0.5f), 0.05f, math::Tolerance<float>());
    EXPECT_NEAR(filter.Filter(0.0f), 0.125f, math::Tolerance<float>());
    EXPECT_NEAR(filter.Filter(0.0f), 0.0575f, math::Tolerance<float>());
}

TEST_F(TestIir, reset_restores_initial_state)
{
    auto b = MakeCoeffs(0.1f, 0.0f);
    auto a = MakeCoeffs(0.5f, 0.4f);
    filters::passive::Iir<float, Order, Order> filter{ b, a };

    filter.Filter(0.5f);
    filter.Filter(0.5f);
    filter.Reset();

    EXPECT_NEAR(filter.Filter(0.5f), 0.05f, math::Tolerance<float>());
}

TEST_F(TestIir, determinism_same_input_twice_gives_identical_output)
{
    auto b = MakeCoeffs(0.1f, 0.2f);
    auto a = MakeCoeffs(0.5f, -0.1f);
    filters::passive::Iir<float, Order, Order> filter1{ b, a };
    filters::passive::Iir<float, Order, Order> filter2{ b, a };

    const float out1a = filter1.Filter(0.5f);
    const float out1b = filter1.Filter(0.3f);
    const float out2a = filter2.Filter(0.5f);
    const float out2b = filter2.Filter(0.3f);

    EXPECT_FLOAT_EQ(out1a, out2a);
    EXPECT_FLOAT_EQ(out1b, out2b);
}

TEST_F(TestIir, zero_feedforward_coefficients_produce_zero_output)
{
    auto b = MakeCoeffs(0.0f, 0.0f);
    auto a = MakeCoeffs(0.5f, 0.0f);
    filters::passive::Iir<float, Order, Order> filter{ b, a };

    EXPECT_FLOAT_EQ(filter.Filter(0.5f), 0.0f);
    EXPECT_FLOAT_EQ(filter.Filter(-0.5f), 0.0f);
}

TEST_F(TestIir, dc_gain_matches_transfer_function)
{
    auto b = MakeCoeffs(0.25f, 0.0f);
    auto a = MakeCoeffs(0.5f, 0.0f);
    filters::passive::Iir<float, Order, Order> filter{ b, a };

    float out = 0.0f;
    for (int i = 0; i < 200; ++i)
        out = filter.Filter(1.0f);

    const float dcGain = 0.25f / (1.0f - 0.5f);
    EXPECT_NEAR(out, dcGain, math::Tolerance<float>());
}

TEST_F(TestIir, impulse_response_decays_for_stable_filter)
{
    auto b = MakeCoeffs(1.0f, 0.0f);
    auto a = MakeCoeffs(-0.3f, -0.2f);
    filters::passive::Iir<float, Order, Order> filter{ b, a };

    filter.Filter(1.0f);
    for (int i = 0; i < 50; ++i)
        filter.Filter(0.0f);

    EXPECT_NEAR(filter.Filter(0.0f), 0.0f, math::Tolerance<float>());
}

TEST_F(TestIir, reset_after_impulse_matches_fresh_instance)
{
    auto b = MakeCoeffs(0.1f, 0.2f);
    auto a = MakeCoeffs(0.5f, -0.1f);
    filters::passive::Iir<float, Order, Order> filter{ b, a };
    filters::passive::Iir<float, Order, Order> fresh{ b, a };

    filter.Filter(1.0f);
    filter.Filter(0.5f);
    filter.Reset();

    EXPECT_FLOAT_EQ(filter.Filter(0.5f), fresh.Filter(0.5f));
    EXPECT_FLOAT_EQ(filter.Filter(0.0f), fresh.Filter(0.0f));
}
