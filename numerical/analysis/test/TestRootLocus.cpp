#include "numerical/analysis/RootLocus.hpp"
#include <array>
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    class TestRootLocus : public ::testing::Test
    {
    protected:
        analysis::RootLocus<float, 5, 100> rootLocus;
    };
}

TEST_F(TestRootLocus, finds_open_loop_poles_of_first_order_system)
{
    std::array<float, 1> num = { 1.0f };
    std::array<float, 2> den = { 1.0f, 2.0f };

    auto result = rootLocus.Calculate(
        infra::MemoryRange<const float>(num),
        infra::MemoryRange<const float>(den),
        1.0f);

    ASSERT_EQ(result.openLoopPoles.size(), 1u);
    EXPECT_NEAR(result.openLoopPoles[0].real(), -2.0f, 1e-3f);
    EXPECT_NEAR(result.openLoopPoles[0].imag(), 0.0f, 1e-3f);
}

TEST_F(TestRootLocus, finds_open_loop_poles_of_second_order_system)
{
    float wn = 2.0f;
    float zeta = 0.5f;
    std::array<float, 1> num = { wn * wn };
    std::array<float, 3> den = { 1.0f, 2.0f * zeta * wn, wn * wn };

    auto result = rootLocus.Calculate(
        infra::MemoryRange<const float>(num),
        infra::MemoryRange<const float>(den),
        1.0f);

    ASSERT_EQ(result.openLoopPoles.size(), 2u);
    EXPECT_NEAR(result.openLoopPoles[0].real(), -zeta * wn, 1e-2f);
    EXPECT_NEAR(result.openLoopPoles[1].real(), -zeta * wn, 1e-2f);
    float expectedImag = wn * std::sqrt(1.0f - zeta * zeta);
    EXPECT_NEAR(std::abs(result.openLoopPoles[0].imag()), expectedImag, 1e-2f);
}

TEST_F(TestRootLocus, finds_open_loop_zeros)
{
    std::array<float, 2> num = { 1.0f, 3.0f };
    std::array<float, 3> den = { 1.0f, 5.0f, 6.0f };

    auto result = rootLocus.Calculate(
        infra::MemoryRange<const float>(num),
        infra::MemoryRange<const float>(den),
        1.0f);

    ASSERT_EQ(result.openLoopZeros.size(), 1u);
    EXPECT_NEAR(result.openLoopZeros[0].real(), -3.0f, 1e-3f);
    EXPECT_NEAR(result.openLoopZeros[0].imag(), 0.0f, 1e-3f);
}

TEST_F(TestRootLocus, gain_sweep_produces_correct_number_of_steps)
{
    std::array<float, 1> num = { 1.0f };
    std::array<float, 2> den = { 1.0f, 1.0f };

    auto result = rootLocus.Calculate(
        infra::MemoryRange<const float>(num),
        infra::MemoryRange<const float>(den),
        1.0f);

    EXPECT_EQ(result.gains.size(), 100u);
    EXPECT_EQ(result.activeBranches, 1u);
    EXPECT_EQ(result.loci[0].size(), 100u);
}

TEST_F(TestRootLocus, gain_sweep_covers_range)
{
    std::array<float, 1> num = { 1.0f };
    std::array<float, 2> den = { 1.0f, 1.0f };

    auto result = rootLocus.Calculate(
        infra::MemoryRange<const float>(num),
        infra::MemoryRange<const float>(den),
        1.0f, 0.01f, 10.0f);

    EXPECT_NEAR(result.gains.front(), 0.01f, 1e-4f);
    EXPECT_NEAR(result.gains.back(), 10.0f, 1e-4f);
}

TEST_F(TestRootLocus, closed_loop_poles_at_current_gain)
{
    std::array<float, 1> num = { 1.0f };
    std::array<float, 2> den = { 1.0f, 1.0f };

    auto result = rootLocus.Calculate(
        infra::MemoryRange<const float>(num),
        infra::MemoryRange<const float>(den),
        2.0f);

    ASSERT_EQ(result.closedLoopPoles.size(), 1u);
    EXPECT_NEAR(result.closedLoopPoles[0].real(), -3.0f, 1e-3f);
    EXPECT_NEAR(result.closedLoopPoles[0].imag(), 0.0f, 1e-3f);
    EXPECT_FLOAT_EQ(result.currentGain, 2.0f);
}

TEST_F(TestRootLocus, loci_move_left_with_increasing_gain_for_first_order)
{
    std::array<float, 1> num = { 1.0f };
    std::array<float, 2> den = { 1.0f, 1.0f };

    auto result = rootLocus.Calculate(
        infra::MemoryRange<const float>(num),
        infra::MemoryRange<const float>(den),
        1.0f, 0.1f, 100.0f);

    ASSERT_GE(result.loci[0].size(), 2u);
    float firstReal = result.loci[0].front().real();
    float lastReal = result.loci[0].back().real();
    EXPECT_LT(lastReal, firstReal);
}

TEST_F(TestRootLocus, second_order_poles_become_complex_with_high_gain)
{
    std::array<float, 1> num = { 1.0f };
    std::array<float, 3> den = { 1.0f, 3.0f, 2.0f };

    auto result = rootLocus.Calculate(
        infra::MemoryRange<const float>(num),
        infra::MemoryRange<const float>(den),
        10.0f, 0.01f, 100.0f);

    ASSERT_EQ(result.activeBranches, 2u);

    bool foundComplex = false;
    for (const auto& root : result.loci[0])
    {
        if (std::abs(root.imag()) > 0.1f)
        {
            foundComplex = true;
            break;
        }
    }
    EXPECT_TRUE(foundComplex);
}

TEST_F(TestRootLocus, stores_current_gain)
{
    std::array<float, 1> num = { 1.0f };
    std::array<float, 2> den = { 1.0f, 1.0f };

    auto result = rootLocus.Calculate(
        infra::MemoryRange<const float>(num),
        infra::MemoryRange<const float>(den),
        42.0f);

    EXPECT_FLOAT_EQ(result.currentGain, 42.0f);
}

TEST_F(TestRootLocus, works_with_pid_and_first_order_plant)
{
    std::array<float, 3> pidNum = { 0.05f, 1.0f, 0.1f };
    std::array<float, 2> pidDen = { 1.0f, 0.0f };

    std::array<float, 1> plantNum = { 1.0f };
    std::array<float, 2> plantDen = { 1.0f, 1.0f };

    std::array<float, 3> openNum;
    openNum[0] = pidNum[0] * plantNum[0];
    openNum[1] = pidNum[1] * plantNum[0];
    openNum[2] = pidNum[2] * plantNum[0];

    std::array<float, 3> openDen;
    openDen[0] = pidDen[0] * plantDen[0];
    openDen[1] = pidDen[0] * plantDen[1] + pidDen[1] * plantDen[0];
    openDen[2] = pidDen[1] * plantDen[1];

    auto result = rootLocus.Calculate(
        infra::MemoryRange<const float>(openNum),
        infra::MemoryRange<const float>(openDen),
        1.0f);

    EXPECT_EQ(result.activeBranches, 2u);
    ASSERT_EQ(result.closedLoopPoles.size(), 2u);
}
