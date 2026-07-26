#include "numerical/controllers/implementations/BangBangHysteresis.hpp"
#include "gtest/gtest.h"

namespace
{
    class TestBangBangHysteresis
        : public ::testing::Test
    {
    public:
        controllers::BangBangHysteresis<float> relay{ -0.2f, 0.2f, 0.0f, 1.0f };
    };
}

TEST_F(TestBangBangHysteresis, starts_in_low_state)
{
    EXPECT_EQ(relay.State(), controllers::RelayState::Low);
    EXPECT_FLOAT_EQ(relay.Update(0.0f), 0.0f);
}

TEST_F(TestBangBangHysteresis, switches_high_at_upper_threshold)
{
    float output = relay.Update(0.2f);

    EXPECT_EQ(relay.State(), controllers::RelayState::High);
    EXPECT_FLOAT_EQ(output, 1.0f);
}

TEST_F(TestBangBangHysteresis, stays_high_inside_band)
{
    relay.Update(0.2f);

    float output = relay.Update(0.0f);

    EXPECT_EQ(relay.State(), controllers::RelayState::High);
    EXPECT_FLOAT_EQ(output, 1.0f);
}

TEST_F(TestBangBangHysteresis, switches_low_at_lower_threshold)
{
    relay.Update(0.2f);

    float output = relay.Update(-0.2f);

    EXPECT_EQ(relay.State(), controllers::RelayState::Low);
    EXPECT_FLOAT_EQ(output, 0.0f);
}

TEST_F(TestBangBangHysteresis, hysteresis_prevents_chatter)
{
    std::array<float, 5> inputs{ 0.1f, -0.1f, 0.15f, -0.15f, 0.05f };

    for (float x : inputs)
        relay.Update(x);

    EXPECT_EQ(relay.State(), controllers::RelayState::Low);
}

TEST_F(TestBangBangHysteresis, full_cycle_sequence)
{
    std::array<float, 5> inputs{ 0.0f, 0.3f, 0.1f, -0.3f, 0.0f };
    std::array<controllers::RelayState, 5> expectedStates{
        controllers::RelayState::Low,
        controllers::RelayState::High,
        controllers::RelayState::High,
        controllers::RelayState::Low,
        controllers::RelayState::Low
    };

    for (std::size_t i = 0; i < inputs.size(); ++i)
    {
        relay.Update(inputs[i]);
        EXPECT_EQ(relay.State(), expectedStates[i]);
    }
}

TEST_F(TestBangBangHysteresis, reset_restores_initial_state)
{
    relay.Update(0.2f);
    EXPECT_EQ(relay.State(), controllers::RelayState::High);

    relay.Reset(controllers::RelayState::Low);

    EXPECT_EQ(relay.State(), controllers::RelayState::Low);
}

TEST_F(TestBangBangHysteresis, custom_output_levels)
{
    controllers::BangBangHysteresis<float> customRelay{ -0.2f, 0.2f, -1.0f, 1.0f };

    EXPECT_FLOAT_EQ(customRelay.Update(0.0f), -1.0f);
    EXPECT_FLOAT_EQ(customRelay.Update(0.2f), 1.0f);
}
