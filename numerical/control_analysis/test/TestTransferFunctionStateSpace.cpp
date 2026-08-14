#include "numerical/control_analysis/TransferFunctionStateSpace.hpp"
#include "numerical/math/Tolerance.hpp"
#include <gtest/gtest.h>

namespace
{
    using TFSS = control_analysis::TransferFunctionStateSpace<float, 2>;
    using TF2 = control_analysis::TransferFunction<float, 2>;
    using LTI = math::LinearTimeInvariant<float, 2, 1, 1>;

    class TestTransferFunctionStateSpace : public ::testing::Test
    {
    protected:
        TF2 tf{};

        void SetUp() override
        {
            tf.denominator = { 1.0f, 3.0f, 2.0f };
            tf.numerator = { 0.0f, 1.0f, 2.0f };
        }
    };
}

TEST_F(TestTransferFunctionStateSpace, controllable_form_matches_hand_value)
{
    auto sys{ TFSS::ToControllableCanonical(tf) };

    EXPECT_NEAR(sys.A.at(0, 0), 0.0f, 1e-5f);
    EXPECT_NEAR(sys.A.at(0, 1), 1.0f, 1e-5f);
    EXPECT_NEAR(sys.A.at(1, 0), -2.0f, 1e-5f);
    EXPECT_NEAR(sys.A.at(1, 1), -3.0f, 1e-5f);
    EXPECT_NEAR(sys.B.at(0, 0), 0.0f, 1e-5f);
    EXPECT_NEAR(sys.B.at(1, 0), 1.0f, 1e-5f);
    EXPECT_NEAR(sys.C.at(0, 0), 2.0f, 1e-5f);
    EXPECT_NEAR(sys.C.at(0, 1), 1.0f, 1e-5f);
    EXPECT_NEAR(sys.D.at(0, 0), 0.0f, 1e-5f);
}

TEST_F(TestTransferFunctionStateSpace, companion_bottom_row_is_negated_denominator)
{
    auto sys{ TFSS::ToControllableCanonical(tf) };

    EXPECT_NEAR(sys.A.at(1, 0), -2.0f, 1e-5f);
    EXPECT_NEAR(sys.A.at(1, 1), -3.0f, 1e-5f);
}

TEST_F(TestTransferFunctionStateSpace, observable_form_is_dual_of_controllable)
{
    auto ccf{ TFSS::ToControllableCanonical(tf) };
    auto ocf{ TFSS::ToObservableCanonical(tf) };

    for (std::size_t r{ 0 }; r < 2; ++r)
        for (std::size_t c{ 0 }; c < 2; ++c)
            EXPECT_NEAR(ocf.A.at(r, c), ccf.A.Transpose().at(r, c), 1e-5f);

    EXPECT_NEAR(ocf.B.at(0, 0), ccf.C.at(0, 0), 1e-5f);
    EXPECT_NEAR(ocf.B.at(1, 0), ccf.C.at(0, 1), 1e-5f);
    EXPECT_NEAR(ocf.C.at(0, 0), ccf.B.at(0, 0), 1e-5f);
    EXPECT_NEAR(ocf.C.at(0, 1), ccf.B.at(1, 0), 1e-5f);
}

TEST_F(TestTransferFunctionStateSpace, state_space_to_tf_recovers_denominator)
{
    auto sys{ TFSS::ToControllableCanonical(tf) };
    auto tf2{ TFSS::ToTransferFunction(sys) };

    EXPECT_NEAR(tf2.denominator[0], 1.0f, 1e-4f);
    EXPECT_NEAR(tf2.denominator[1], 3.0f, 1e-4f);
    EXPECT_NEAR(tf2.denominator[2], 2.0f, 1e-4f);
}

TEST_F(TestTransferFunctionStateSpace, round_trip_tf_ss_tf_is_identity)
{
    auto tf2{ TFSS::ToTransferFunction(TFSS::ToControllableCanonical(tf)) };

    EXPECT_NEAR(tf2.numerator[0], 0.0f, 1e-4f);
    EXPECT_NEAR(tf2.numerator[1], 1.0f, 1e-4f);
    EXPECT_NEAR(tf2.numerator[2], 2.0f, 1e-4f);
    EXPECT_NEAR(tf2.denominator[0], 1.0f, 1e-4f);
    EXPECT_NEAR(tf2.denominator[1], 3.0f, 1e-4f);
    EXPECT_NEAR(tf2.denominator[2], 2.0f, 1e-4f);
}

TEST_F(TestTransferFunctionStateSpace, feedthrough_extracted_for_proper_tf)
{
    TF2 proper{};
    proper.denominator = { 1.0f, 3.0f, 2.0f };
    proper.numerator = { 2.0f, 3.0f, 4.0f };

    auto sys{ TFSS::ToControllableCanonical(proper) };

    EXPECT_NEAR(sys.D.at(0, 0), 2.0f, 1e-5f);
    EXPECT_NEAR(sys.C.at(0, 0), 0.0f, 1e-4f);
    EXPECT_NEAR(sys.C.at(0, 1), -3.0f, 1e-4f);
}

TEST_F(TestTransferFunctionStateSpace, strictly_proper_has_zero_feedthrough)
{
    auto sys{ TFSS::ToControllableCanonical(tf) };

    EXPECT_NEAR(sys.D.at(0, 0), 0.0f, 1e-5f);
}

TEST_F(TestTransferFunctionStateSpace, denominator_normalized_to_monic)
{
    TF2 nonMonic{};
    nonMonic.denominator = { 2.0f, 6.0f, 4.0f };
    nonMonic.numerator = { 0.0f, 2.0f, 4.0f };

    auto sys1{ TFSS::ToControllableCanonical(tf) };
    auto sys2{ TFSS::ToControllableCanonical(nonMonic) };

    for (std::size_t r{ 0 }; r < 2; ++r)
        for (std::size_t c{ 0 }; c < 2; ++c)
            EXPECT_NEAR(sys1.A.at(r, c), sys2.A.at(r, c), 1e-5f);
    EXPECT_NEAR(sys1.C.at(0, 0), sys2.C.at(0, 0), 1e-5f);
    EXPECT_NEAR(sys1.C.at(0, 1), sys2.C.at(0, 1), 1e-5f);
}

TEST_F(TestTransferFunctionStateSpace, both_forms_share_impulse_response)
{
    auto ccf{ TFSS::ToControllableCanonical(tf) };
    auto ocf{ TFSS::ToObservableCanonical(tf) };

    math::Matrix<float, 2, 1> xCcf{};
    math::Matrix<float, 2, 1> xOcf{};
    math::Matrix<float, 1, 1> u{};
    u.at(0, 0) = 1.0f;

    constexpr std::size_t K{ 5 };
    for (std::size_t k{ 0 }; k < K; ++k)
    {
        auto yCcf{ ccf.Output(xCcf, u) };
        auto yOcf{ ocf.Output(xOcf, u) };
        EXPECT_NEAR(yCcf.at(0, 0), yOcf.at(0, 0), 1e-4f);
        xCcf = ccf.Step(xCcf, u);
        xOcf = ocf.Step(xOcf, u);
        u.at(0, 0) = 0.0f;
    }
}

TEST_F(TestTransferFunctionStateSpace, round_trip_via_observable_canonical_is_identity)
{
    auto tf2{ TFSS::ToTransferFunction(TFSS::ToObservableCanonical(tf)) };

    EXPECT_NEAR(tf2.numerator[0], 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(tf2.numerator[1], 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(tf2.numerator[2], 2.0f, math::Tolerance<float>());
    EXPECT_NEAR(tf2.denominator[0], 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(tf2.denominator[1], 3.0f, math::Tolerance<float>());
    EXPECT_NEAR(tf2.denominator[2], 2.0f, math::Tolerance<float>());
}

TEST_F(TestTransferFunctionStateSpace, non_monic_round_trip_recovers_monic_tf)
{
    TF2 nonMonic{};
    nonMonic.denominator = { 2.0f, 6.0f, 4.0f };
    nonMonic.numerator = { 0.0f, 2.0f, 4.0f };

    auto tf2{ TFSS::ToTransferFunction(TFSS::ToControllableCanonical(nonMonic)) };

    EXPECT_NEAR(tf2.numerator[0], 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(tf2.numerator[1], 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(tf2.numerator[2], 2.0f, math::Tolerance<float>());
    EXPECT_NEAR(tf2.denominator[0], 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(tf2.denominator[1], 3.0f, math::Tolerance<float>());
    EXPECT_NEAR(tf2.denominator[2], 2.0f, math::Tolerance<float>());
}

TEST_F(TestTransferFunctionStateSpace, round_trip_with_nonzero_feedthrough_recovers_numerator)
{
    TF2 improper{};
    improper.denominator = { 1.0f, 3.0f, 2.0f };
    improper.numerator = { 2.0f, 7.0f, 9.0f };

    auto tf2{ TFSS::ToTransferFunction(TFSS::ToControllableCanonical(improper)) };

    EXPECT_NEAR(tf2.numerator[0], 2.0f, 1e-4f);
    EXPECT_NEAR(tf2.numerator[1], 7.0f, 1e-4f);
    EXPECT_NEAR(tf2.numerator[2], 9.0f, 1e-4f);
    EXPECT_NEAR(tf2.denominator[0], 1.0f, 1e-4f);
    EXPECT_NEAR(tf2.denominator[1], 3.0f, 1e-4f);
    EXPECT_NEAR(tf2.denominator[2], 2.0f, 1e-4f);
}

TEST_F(TestTransferFunctionStateSpace, dc_gain_correct_for_system_with_feedthrough)
{
    TF2 improper{};
    improper.denominator = { 1.0f, 1.0f, 0.0f };
    improper.numerator = { 2.0f, 3.0f, 0.0f };

    auto sys{ TFSS::ToControllableCanonical(improper) };
    auto tf2{ TFSS::ToTransferFunction(sys) };

    EXPECT_NEAR(tf2.numerator[0], 2.0f, 1e-4f);
    EXPECT_NEAR(tf2.numerator[1], 3.0f, 1e-4f);
}
