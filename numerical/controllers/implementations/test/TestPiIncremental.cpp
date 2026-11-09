#include "numerical/controllers/implementations/PiIncremental.hpp"
#include "gtest/gtest.h"

namespace
{
    class TestPiIncremental
        : public ::testing::Test
    {
    public:
        static constexpr float tolerance{ 1e-5f };

        controllers::PiIncremental<float> pi;
    };
}

TEST_F(TestPiIncremental, untuned_controller_proposes_zero)
{
    EXPECT_NEAR(pi.Propose(5.0f, 0.0f), 0.0f, tolerance);
}

TEST_F(TestPiIncremental, first_proposal_is_the_proportional_plus_integral_step)
{
    pi.SetTunings(2.0f, 0.5f);

    EXPECT_NEAR(pi.Propose(5.0f, 1.0f), 10.0f, tolerance);
}

TEST_F(TestPiIncremental, constant_error_grows_the_output_by_the_integral_step)
{
    pi.SetTunings(2.0f, 0.5f);

    pi.Propose(1.0f, 0.0f);

    EXPECT_NEAR(pi.Propose(1.0f, 0.0f), 3.0f, tolerance);
    EXPECT_NEAR(pi.Propose(1.0f, 0.0f), 3.5f, tolerance);
}

TEST_F(TestPiIncremental, follows_the_velocity_form_law)
{
    pi.SetTunings(1.5f, 0.25f);

    const float errors[]{ 2.0f, -1.0f, 0.5f, 3.0f };
    float expected{ 0.0f };
    float previousError{ 0.0f };

    for (const auto error : errors)
    {
        expected += (1.5f + 0.25f) * error - 1.5f * previousError;
        previousError = error;

        EXPECT_NEAR(pi.Propose(error, 0.0f), expected, tolerance);
    }
}

TEST_F(TestPiIncremental, committing_a_realized_value_back_calculates_the_next_proposal)
{
    pi.SetTunings(2.0f, 0.0f);

    pi.Propose(5.0f, 0.0f);
    pi.CommitRealized(6.0f);

    EXPECT_NEAR(pi.Propose(5.0f, 0.0f), 6.0f, tolerance);
}

TEST_F(TestPiIncremental, without_a_commit_the_next_proposal_builds_on_the_raw_output)
{
    pi.SetTunings(2.0f, 0.0f);

    pi.Propose(5.0f, 0.0f);

    EXPECT_NEAR(pi.Propose(5.0f, 0.0f), 10.0f, tolerance);
}

TEST_F(TestPiIncremental, saturated_commits_keep_the_integrator_from_winding_up)
{
    pi.SetTunings(1.0f, 0.5f);

    for (int i = 0; i < 100; ++i)
    {
        pi.Propose(10.0f, 0.0f);
        pi.CommitRealized(1.0f);
    }

    EXPECT_NEAR(pi.Propose(10.0f, 0.0f), 6.0f, tolerance);
}

TEST_F(TestPiIncremental, reset_clears_the_output_and_the_error_history)
{
    pi.SetTunings(2.0f, 0.0f);

    pi.Propose(5.0f, 0.0f);
    pi.CommitRealized(6.0f);
    pi.Reset();

    EXPECT_NEAR(pi.Propose(5.0f, 0.0f), 10.0f, tolerance);
}

TEST_F(TestPiIncremental, new_tunings_keep_the_accumulated_output)
{
    pi.SetTunings(1.0f, 1.0f);
    pi.Propose(1.0f, 0.0f);

    pi.SetTunings(0.0f, 0.0f);

    EXPECT_NEAR(pi.Propose(3.0f, 0.0f), 2.0f, tolerance);
}
