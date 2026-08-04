#include "numerical/control_analysis/RootLocus.hpp"
#include "numerical/math/Tolerance.hpp"
#include <array>
#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    class TestRootLocus : public ::testing::Test
    {
    protected:
        control_analysis::RootLocus<float, 5, 100> rootLocus;
    };
}

TEST_F(TestRootLocus, open_loop_poles_first_order_plant)
{
    std::array<float, 1> num = { 1.0f };
    std::array<float, 2> den = { 1.0f, 2.0f };

    auto result = rootLocus.Calculate(num, den, 1.0f);

    ASSERT_EQ(result.openLoopPoles.size(), 1u);
    EXPECT_NEAR(result.openLoopPoles[0].Real(), -2.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.openLoopPoles[0].Imaginary(), 0.0f, math::Tolerance<float>());
}

TEST_F(TestRootLocus, open_loop_poles_second_order_underdamped)
{
    constexpr float wn = 2.0f;
    constexpr float zeta = 0.5f;
    std::array<float, 1> num = { 1.0f };
    std::array<float, 3> den = { 1.0f, 2.0f * zeta * wn, wn * wn };

    auto result = rootLocus.Calculate(num, den, 1.0f);

    ASSERT_EQ(result.openLoopPoles.size(), 2u);
    EXPECT_NEAR(result.openLoopPoles[0].Real(), -zeta * wn, 1e-2f);
    EXPECT_NEAR(result.openLoopPoles[1].Real(), -zeta * wn, 1e-2f);
    float expectedImag = wn * std::sqrt(1.0f - zeta * zeta);
    EXPECT_NEAR(std::abs(result.openLoopPoles[0].Imaginary()), expectedImag, 1e-2f);
}

TEST_F(TestRootLocus, open_loop_zeros_identified)
{
    std::array<float, 2> num = { 1.0f, 3.0f };
    std::array<float, 3> den = { 1.0f, 5.0f, 6.0f };

    auto result = rootLocus.Calculate(num, den, 1.0f);

    ASSERT_EQ(result.openLoopZeros.size(), 1u);
    EXPECT_NEAR(result.openLoopZeros[0].Real(), -3.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.openLoopZeros[0].Imaginary(), 0.0f, math::Tolerance<float>());
}

TEST_F(TestRootLocus, gain_sweep_step_count_and_active_branches)
{
    std::array<float, 1> num = { 1.0f };
    std::array<float, 2> den = { 1.0f, 1.0f };

    auto result = rootLocus.Calculate(num, den, 1.0f);

    EXPECT_EQ(result.gains.size(), 100u);
    EXPECT_EQ(result.activeBranches, 1u);
    EXPECT_EQ(result.loci[0].size(), 100u);
}

TEST_F(TestRootLocus, gain_sweep_endpoints_match_requested_range)
{
    std::array<float, 1> num = { 1.0f };
    std::array<float, 2> den = { 1.0f, 1.0f };

    auto result = rootLocus.Calculate(num, den, 1.0f, 0.01f, 10.0f);

    EXPECT_NEAR(result.gains.front(), 0.01f, 1e-4f);
    EXPECT_NEAR(result.gains.back(), 10.0f, 1e-4f);
}

TEST_F(TestRootLocus, closed_loop_poles_match_analytic_at_k1_doc_example)
{
    std::array<float, 2> num = { 1.0f, 1.0f };
    std::array<float, 3> den = { 1.0f, 2.0f, 0.0f };

    auto result = rootLocus.Calculate(num, den, 1.0f);

    ASSERT_EQ(result.closedLoopPoles.size(), 2u);
    float r0 = (-3.0f - std::sqrt(5.0f)) / 2.0f;
    float r1 = (-3.0f + std::sqrt(5.0f)) / 2.0f;
    EXPECT_NEAR(result.closedLoopPoles[0].Real(), r0, 1e-2f);
    EXPECT_NEAR(result.closedLoopPoles[0].Imaginary(), 0.0f, 1e-2f);
    EXPECT_NEAR(result.closedLoopPoles[1].Real(), r1, 1e-2f);
    EXPECT_NEAR(result.closedLoopPoles[1].Imaginary(), 0.0f, 1e-2f);
}

TEST_F(TestRootLocus, current_gain_stored_in_result)
{
    std::array<float, 1> num = { 1.0f };
    std::array<float, 2> den = { 1.0f, 1.0f };

    auto result = rootLocus.Calculate(num, den, 2.0f);

    EXPECT_FLOAT_EQ(result.currentGain, 2.0f);
}

TEST_F(TestRootLocus, branches_start_near_open_loop_poles_at_low_gain)
{
    std::array<float, 1> num = { 1.0f };
    std::array<float, 3> den = { 1.0f, 3.0f, 2.0f };

    auto result = rootLocus.Calculate(num, den, 0.001f, 0.001f, 50.0f);

    ASSERT_GE(result.loci[0].size(), 1u);
    ASSERT_GE(result.loci[1].size(), 1u);

    float p0 = result.openLoopPoles[0].Real();
    float p1 = result.openLoopPoles[1].Real();

    float loci0Start = result.loci[0].front().Real();
    float loci1Start = result.loci[1].front().Real();

    bool branch0NearP0 = std::abs(loci0Start - p0) < 0.5f;
    bool branch0NearP1 = std::abs(loci0Start - p1) < 0.5f;
    bool branch1NearP0 = std::abs(loci1Start - p0) < 0.5f;
    bool branch1NearP1 = std::abs(loci1Start - p1) < 0.5f;

    EXPECT_TRUE((branch0NearP0 && branch1NearP1) || (branch0NearP1 && branch1NearP0));
}

TEST_F(TestRootLocus, asymptote_centroid_three_poles_one_zero)
{
    std::array<float, 2> num = { 1.0f, 1.0f };
    std::array<float, 4> den = { 1.0f, 6.0f, 11.0f, 6.0f };

    auto result = rootLocus.Calculate(num, den, 1.0f);

    ASSERT_EQ(result.openLoopPoles.size(), 3u);
    ASSERT_EQ(result.openLoopZeros.size(), 1u);

    float sumPoles = 0.0f;
    for (const auto& p : result.openLoopPoles)
        sumPoles += p.Real();

    float sumZeros = 0.0f;
    for (const auto& z : result.openLoopZeros)
        sumZeros += z.Real();

    float centroid = (sumPoles - sumZeros) / static_cast<float>(result.openLoopPoles.size() - result.openLoopZeros.size());

    EXPECT_NEAR(sumPoles, -6.0f, math::Tolerance<float>());
    EXPECT_NEAR(sumZeros, -1.0f, math::Tolerance<float>());
    EXPECT_NEAR(centroid, -2.5f, math::Tolerance<float>());
}

TEST_F(TestRootLocus, asymptote_angles_two_poles_no_zeros)
{
    std::array<float, 1> num = { 1.0f };
    std::array<float, 3> den = { 1.0f, 3.0f, 2.0f };

    auto result = rootLocus.Calculate(num, den, 1.0f);

    int excessPoles = static_cast<int>(result.openLoopPoles.size()) - static_cast<int>(result.openLoopZeros.size());
    ASSERT_EQ(excessPoles, 2);

    float angle0 = std::numbers::pi_v<float> / static_cast<float>(excessPoles);
    float angle1 = 3.0f * std::numbers::pi_v<float> / static_cast<float>(excessPoles);

    EXPECT_NEAR(angle0, std::numbers::pi_v<float> / 2.0f, math::Tolerance<float>());
    EXPECT_NEAR(angle1, 3.0f * std::numbers::pi_v<float> / 2.0f, math::Tolerance<float>());
}

TEST_F(TestRootLocus, loci_move_left_with_increasing_gain_first_order)
{
    std::array<float, 1> num = { 1.0f };
    std::array<float, 2> den = { 1.0f, 1.0f };

    auto result = rootLocus.Calculate(num, den, 1.0f, 0.1f, 100.0f);

    ASSERT_GE(result.loci[0].size(), 2u);
    EXPECT_LT(result.loci[0].back().Real(), result.loci[0].front().Real());
}

TEST_F(TestRootLocus, second_order_poles_become_complex_at_high_gain)
{
    std::array<float, 1> num = { 1.0f };
    std::array<float, 3> den = { 1.0f, 3.0f, 2.0f };

    auto result = rootLocus.Calculate(num, den, 10.0f, 0.01f, 100.0f);

    ASSERT_EQ(result.activeBranches, 2u);

    bool foundComplex = false;
    for (const auto& root : result.loci[0])
    {
        if (std::abs(root.Imaginary()) > 0.1f)
        {
            foundComplex = true;
            break;
        }
    }
    for (const auto& root : result.loci[1])
    {
        if (std::abs(root.Imaginary()) > 0.1f)
        {
            foundComplex = true;
            break;
        }
    }
    EXPECT_TRUE(foundComplex);
}

TEST_F(TestRootLocus, conjugate_symmetry_when_complex_pair_present)
{
    std::array<float, 1> num = { 1.0f };
    std::array<float, 3> den = { 1.0f, 3.0f, 2.0f };

    auto result = rootLocus.Calculate(num, den, 5.0f, 0.01f, 100.0f);

    ASSERT_EQ(result.activeBranches, 2u);

    bool verified = false;
    for (std::size_t i = 0; i < result.loci[0].size(); ++i)
    {
        float im0 = result.loci[0][i].Imaginary();
        float im1 = result.loci[1][i].Imaginary();
        if (std::abs(im0) > 0.1f && std::abs(im1) > 0.1f)
        {
            float re0 = result.loci[0][i].Real();
            float re1 = result.loci[1][i].Real();
            EXPECT_NEAR(re0, re1, 1e-2f);
            EXPECT_NEAR(im0, -im1, 1e-2f);
            verified = true;
        }
    }
    EXPECT_TRUE(verified);
}

TEST_F(TestRootLocus, closed_loop_poles_in_lhp_for_stable_gain)
{
    std::array<float, 1> num = { 1.0f };
    std::array<float, 3> den = { 1.0f, 3.0f, 2.0f };

    auto result = rootLocus.Calculate(num, den, 1.0f);

    for (const auto& p : result.closedLoopPoles)
        EXPECT_LT(p.Real(), 0.0f);
}

TEST_F(TestRootLocus, all_loci_points_are_finite)
{
    std::array<float, 1> num = { 1.0f };
    std::array<float, 3> den = { 1.0f, 3.0f, 2.0f };

    auto result = rootLocus.Calculate(num, den, 1.0f, 0.001f, 50.0f);

    for (std::size_t b = 0; b < result.activeBranches; ++b)
    {
        for (const auto& pt : result.loci[b])
        {
            EXPECT_TRUE(std::isfinite(pt.Real()));
            EXPECT_TRUE(std::isfinite(pt.Imaginary()));
        }
    }
}

TEST_F(TestRootLocus, determinism_same_input_same_output)
{
    std::array<float, 1> num = { 1.0f };
    std::array<float, 3> den = { 1.0f, 3.0f, 2.0f };

    auto result1 = rootLocus.Calculate(num, den, 2.0f, 0.01f, 50.0f);
    auto result2 = rootLocus.Calculate(num, den, 2.0f, 0.01f, 50.0f);

    ASSERT_EQ(result1.activeBranches, result2.activeBranches);
    for (std::size_t b = 0; b < result1.activeBranches; ++b)
    {
        ASSERT_EQ(result1.loci[b].size(), result2.loci[b].size());
        for (std::size_t i = 0; i < result1.loci[b].size(); ++i)
        {
            EXPECT_FLOAT_EQ(result1.loci[b][i].Real(), result2.loci[b][i].Real());
            EXPECT_FLOAT_EQ(result1.loci[b][i].Imaginary(), result2.loci[b][i].Imaginary());
        }
    }
}

TEST_F(TestRootLocus, constant_numerator_produces_no_zeros)
{
    std::array<float, 1> num = { 1.0f };
    std::array<float, 3> den = { 1.0f, 5.0f, 6.0f };

    auto result = rootLocus.Calculate(num, den, 1.0f);

    EXPECT_EQ(result.openLoopZeros.size(), 0u);
    EXPECT_EQ(result.activeBranches, 2u);
}

TEST_F(TestRootLocus, three_poles_two_zeros_active_branches)
{
    std::array<float, 3> num = { 1.0f, 3.0f, 2.0f };
    std::array<float, 4> den = { 1.0f, 6.0f, 11.0f, 6.0f };

    auto result = rootLocus.Calculate(num, den, 1.0f);

    EXPECT_EQ(result.activeBranches, 3u);
    EXPECT_EQ(result.openLoopPoles.size(), 3u);
    EXPECT_EQ(result.openLoopZeros.size(), 2u);
}

TEST_F(TestRootLocus, closed_loop_pole_first_order_analytic)
{
    std::array<float, 1> num = { 1.0f };
    std::array<float, 2> den = { 1.0f, 1.0f };

    auto result = rootLocus.Calculate(num, den, 2.0f);

    ASSERT_EQ(result.closedLoopPoles.size(), 1u);
    EXPECT_NEAR(result.closedLoopPoles[0].Real(), -3.0f, math::Tolerance<float>());
    EXPECT_NEAR(result.closedLoopPoles[0].Imaginary(), 0.0f, math::Tolerance<float>());
}
