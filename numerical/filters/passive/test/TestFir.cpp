// Copyright (c) 2025 numerical-toolbox contributors. SPDX-License-Identifier: MIT
#include "numerical/filters/passive/Fir.hpp"
#include "numerical/math/RecursiveBuffer.hpp"
#include "numerical/math/Tolerance.hpp"
#include <array>
#include <cmath>
#include <numbers>
#include <gtest/gtest.h>

namespace
{
    class TestFir : public ::testing::Test
    {
    protected:
        static constexpr std::size_t N = 3;

        math::RecursiveBuffer<float, N> MakeCoeffs(float b0, float b1, float b2) noexcept
        {
            math::RecursiveBuffer<float, N> c;
            c = { b0, b1, b2 };
            return c;
        }
    };
}

TEST_F(TestFir, ImpulseResponseEqualsCoefficients)
{
    auto coeffs = MakeCoeffs(0.3f, 0.5f, 0.2f);
    filters::passive::Fir<float, N> fir{ coeffs };

    EXPECT_NEAR(fir.Filter(1.0f), 0.3f, math::Tolerance<float>());
    EXPECT_NEAR(fir.Filter(0.0f), 0.5f, math::Tolerance<float>());
    EXPECT_NEAR(fir.Filter(0.0f), 0.2f, math::Tolerance<float>());
    EXPECT_NEAR(fir.Filter(0.0f), 0.0f, math::Tolerance<float>());
}

TEST_F(TestFir, StepSteadyStateEqualsDcGain)
{
    auto coeffs = MakeCoeffs(0.3f, 0.5f, 0.2f);
    filters::passive::Fir<float, N> fir{ coeffs };

    for (int i = 0; i < 10; ++i)
        fir.Filter(1.0f);

    const float dcGain = 0.3f + 0.5f + 0.2f;
    EXPECT_NEAR(fir.Filter(1.0f), dcGain, math::Tolerance<float>());
}

TEST_F(TestFir, SimpleMovingAverageConvolution)
{
    auto coeffs = MakeCoeffs(0.25f, 0.25f, 0.25f);
    filters::passive::Fir<float, N> fir{ coeffs };

    EXPECT_NEAR(fir.Filter(0.4f), 0.1f, math::Tolerance<float>());
    EXPECT_NEAR(fir.Filter(0.6f), 0.25f, math::Tolerance<float>());
    EXPECT_NEAR(fir.Filter(0.2f), 0.3f, math::Tolerance<float>());
}

TEST_F(TestFir, WeightedConvolutionReference)
{
    auto coeffs = MakeCoeffs(0.3f, 0.2f, 0.1f);
    filters::passive::Fir<float, N> fir{ coeffs };

    EXPECT_NEAR(fir.Filter(0.6f), 0.18f, math::Tolerance<float>());
    EXPECT_NEAR(fir.Filter(0.4f), 0.24f, math::Tolerance<float>());
    EXPECT_NEAR(fir.Filter(0.5f), 0.29f, math::Tolerance<float>());
}

TEST_F(TestFir, ZeroCoefficientsProduceZeroOutput)
{
    auto coeffs = MakeCoeffs(0.0f, 0.0f, 0.0f);
    filters::passive::Fir<float, N> fir{ coeffs };

    EXPECT_FLOAT_EQ(fir.Filter(0.4f), 0.0f);
    EXPECT_FLOAT_EQ(fir.Filter(-0.4f), 0.0f);
}

TEST_F(TestFir, ZeroInputProducesZeroOutput)
{
    auto coeffs = MakeCoeffs(0.3f, 0.5f, 0.2f);
    filters::passive::Fir<float, N> fir{ coeffs };

    EXPECT_FLOAT_EQ(fir.Filter(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(fir.Filter(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(fir.Filter(0.0f), 0.0f);
}

TEST_F(TestFir, ResetRestoresInitialState)
{
    auto coeffs = MakeCoeffs(0.3f, 0.2f, 0.1f);
    filters::passive::Fir<float, N> fir{ coeffs };

    fir.Filter(0.4f);
    fir.Filter(0.6f);
    fir.Reset();

    EXPECT_NEAR(fir.Filter(0.4f), 0.12f, math::Tolerance<float>());
}

TEST_F(TestFir, DeterminismAfterReset)
{
    auto coeffs = MakeCoeffs(0.3f, 0.2f, 0.1f);
    filters::passive::Fir<float, N> firA{ coeffs };
    filters::passive::Fir<float, N> firB{ coeffs };

    firA.Filter(0.7f);
    firA.Filter(0.3f);
    firA.Reset();

    const float outA0 = firA.Filter(0.5f);
    const float outA1 = firA.Filter(0.8f);
    const float outB0 = firB.Filter(0.5f);
    const float outB1 = firB.Filter(0.8f);

    EXPECT_FLOAT_EQ(outA0, outB0);
    EXPECT_FLOAT_EQ(outA1, outB1);
}

TEST_F(TestFir, NyquistAttenuationBelowDcGain)
{
    auto coeffs = MakeCoeffs(1.0f / 3.0f, 1.0f / 3.0f, 1.0f / 3.0f);
    filters::passive::Fir<float, N> fir{ coeffs };

    constexpr int Warmup = 64;
    constexpr int Measure = 64;
    const float pi = std::numbers::pi_v<float>;

    for (int k = 0; k < Warmup; ++k)
        fir.Filter(std::sin(pi * static_cast<float>(k)));

    float rmsOut = 0.0f;
    for (int k = 0; k < Measure; ++k)
    {
        const float y = fir.Filter(std::sin(pi * static_cast<float>(Warmup + k)));
        rmsOut += y * y;
    }
    rmsOut = std::sqrt(rmsOut / static_cast<float>(Measure));

    EXPECT_LT(rmsOut, 0.1f);
}

TEST_F(TestFir, DcPassthroughEqualsGain)
{
    auto coeffs = MakeCoeffs(0.25f, 0.25f, 0.25f);
    filters::passive::Fir<float, N> fir{ coeffs };

    for (int k = 0; k < 20; ++k)
        fir.Filter(1.0f);

    const float dcGain = 0.25f + 0.25f + 0.25f;
    EXPECT_NEAR(fir.Filter(1.0f), dcGain, math::Tolerance<float>());
}

TEST_F(TestFir, LinearitySuperpositionProperty)
{
    auto coeffs = MakeCoeffs(0.3f, 0.2f, 0.1f);
    filters::passive::Fir<float, N> firA{ coeffs };
    filters::passive::Fir<float, N> firB{ coeffs };
    filters::passive::Fir<float, N> firC{ coeffs };

    constexpr std::array<float, 5> x{ 0.1f, -0.3f, 0.7f, 0.0f, 0.5f };
    constexpr std::array<float, 5> z{ 0.4f,  0.2f, -0.1f, 0.6f, -0.2f };
    constexpr float alpha = 2.0f;
    constexpr float beta = -0.5f;

    for (std::size_t k = 0; k < x.size(); ++k)
    {
        const float outSum = firA.Filter(alpha * x[k] + beta * z[k]);
        const float outScaledX = alpha * firB.Filter(x[k]);
        const float outScaledZ = beta * firC.Filter(z[k]);
        EXPECT_NEAR(outSum, outScaledX + outScaledZ, math::Tolerance<float>());
    }
}
