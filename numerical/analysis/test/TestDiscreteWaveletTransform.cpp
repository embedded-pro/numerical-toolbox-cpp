#include "numerical/analysis/DiscreteWaveletTransform.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gtest/gtest.h"
#include <cstddef>
#include <random>

namespace
{
    class TestDiscreteWaveletTransform : public ::testing::Test
    {
    public:
        static constexpr std::size_t N = 16;

        analysis::WaveletFilters<float, 2> haar{ analysis::MakeHaar<float, 2>() };
        analysis::WaveletFilters<float, 4> db2{ analysis::MakeDaubechies2<float, 4>() };

        analysis::DiscreteWaveletTransform<float, N, 3, 2> dwtHaar{ haar };
        analysis::DiscreteWaveletTransform<float, N, 3, 4> dwtDb2{ db2 };

        using Signal16 = typename infra::BoundedVector<float>::WithMaxSize<N>;
        using Signal4 = typename infra::BoundedVector<float>::WithMaxSize<4>;
    };
}

TEST_F(TestDiscreteWaveletTransform, haar_constant_signal_all_details_zero_approx_reference)
{
    Signal16 x;
    x.resize(N, 1.0f);

    Signal16 coeffs;
    dwtHaar.Forward(x, coeffs);

    for (std::size_t lvl = 0; lvl < 3; ++lvl)
    {
        std::size_t offset = dwtHaar.LevelOffset(lvl);
        std::size_t halfLen = N >> (lvl + 1);
        for (std::size_t i = 0; i < halfLen; ++i)
            EXPECT_NEAR(coeffs[offset + i], 0.0f, math::Tolerance<float>());
    }

    constexpr float kExpectedApprox = 2.8284271247f;
    EXPECT_NEAR(coeffs[14], kExpectedApprox, math::Tolerance<float>());
    EXPECT_NEAR(coeffs[15], kExpectedApprox, math::Tolerance<float>());
}

TEST_F(TestDiscreteWaveletTransform, haar_single_level_impulse_reference)
{
    analysis::DiscreteWaveletTransform<float, N, 1, 2> dwt1{ haar };

    Signal16 x;
    x.push_back(1.0f);
    for (std::size_t i = 1; i < N; ++i)
        x.push_back(0.0f);

    Signal16 coeffs;
    dwt1.Forward(x, coeffs);

    constexpr float kInvSqrt2 = 0.7071067811865476f;
    EXPECT_NEAR(coeffs[0], kInvSqrt2, math::Tolerance<float>());
    for (std::size_t i = 1; i < N / 2; ++i)
        EXPECT_NEAR(coeffs[i], 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(coeffs[N / 2], kInvSqrt2, math::Tolerance<float>());
    for (std::size_t i = N / 2 + 1; i < N; ++i)
        EXPECT_NEAR(coeffs[i], 0.0f, math::Tolerance<float>());
}

TEST_F(TestDiscreteWaveletTransform, haar_single_level_qmf_reference)
{
    analysis::DiscreteWaveletTransform<float, 4, 1, 2> dwt1{ haar };

    Signal4 x;
    x.push_back(1.0f);
    x.push_back(2.0f);
    x.push_back(3.0f);
    x.push_back(4.0f);

    Signal4 coeffs;
    dwt1.Forward(x, coeffs);

    constexpr float kInvSqrt2 = 0.7071067811865476f;
    EXPECT_NEAR(coeffs[0], (1.0f - 2.0f) * kInvSqrt2, 1e-5f);
    EXPECT_NEAR(coeffs[1], (3.0f - 4.0f) * kInvSqrt2, 1e-5f);
    EXPECT_NEAR(coeffs[2], (1.0f + 2.0f) * kInvSqrt2, 1e-5f);
    EXPECT_NEAR(coeffs[3], (3.0f + 4.0f) * kInvSqrt2, 1e-5f);
}

TEST_F(TestDiscreteWaveletTransform, db2_single_level_qmf_reference)
{
    analysis::DiscreteWaveletTransform<float, 4, 1, 4> dwt1{ db2 };

    Signal4 x;
    x.push_back(1.0f);
    x.push_back(2.0f);
    x.push_back(3.0f);
    x.push_back(4.0f);

    Signal4 coeffs;
    dwt1.Forward(x, coeffs);

    EXPECT_NEAR(coeffs[0], 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(coeffs[1], -1.4142135624f, math::Tolerance<float>());
    EXPECT_NEAR(coeffs[2], 2.3107890345f, math::Tolerance<float>());
    EXPECT_NEAR(coeffs[3], 4.7602787773f, math::Tolerance<float>());
}

TEST_F(TestDiscreteWaveletTransform, zero_signal_produces_zero_coefficients)
{
    Signal16 x;
    x.resize(N, 0.0f);

    Signal16 coeffs;
    dwtHaar.Forward(x, coeffs);

    for (std::size_t i = 0; i < N; ++i)
        EXPECT_NEAR(coeffs[i], 0.0f, math::Tolerance<float>());
}

TEST_F(TestDiscreteWaveletTransform, zero_coefficients_reconstruct_zero)
{
    Signal16 coeffs;
    coeffs.resize(N, 0.0f);

    Signal16 xRec;
    dwtHaar.Inverse(coeffs, xRec);

    for (std::size_t i = 0; i < N; ++i)
        EXPECT_NEAR(xRec[i], 0.0f, math::Tolerance<float>());
}

TEST_F(TestDiscreteWaveletTransform, haar_detail_captures_step_edge)
{
    Signal16 x;
    for (std::size_t i = 0; i < 5; ++i)
        x.push_back(0.0f);
    for (std::size_t i = 5; i < N; ++i)
        x.push_back(1.0f);

    Signal16 coeffs;
    dwtHaar.Forward(x, coeffs);

    constexpr float kInvSqrt2 = 0.7071067811865476f;
    std::size_t offset0 = dwtHaar.LevelOffset(0);
    EXPECT_NEAR(coeffs[offset0 + 2], -kInvSqrt2, math::Tolerance<float>());
}

TEST_F(TestDiscreteWaveletTransform, haar_energy_preserved_parseval)
{
    Signal16 x;
    for (std::size_t i = 0; i < N; ++i)
        x.push_back(static_cast<float>(i) * 0.1f + 0.5f);

    Signal16 coeffs;
    dwtHaar.Forward(x, coeffs);

    float energyIn{ 0.0f };
    float energyOut{ 0.0f };
    for (std::size_t i = 0; i < N; ++i)
    {
        energyIn += x[i] * x[i];
        energyOut += coeffs[i] * coeffs[i];
    }

    EXPECT_NEAR(energyIn, energyOut, math::Tolerance<float>());
}

TEST_F(TestDiscreteWaveletTransform, db2_energy_preserved_parseval)
{
    Signal16 x;
    for (std::size_t i = 0; i < N; ++i)
        x.push_back(static_cast<float>(i) * 0.1f);

    Signal16 coeffs;
    dwtDb2.Forward(x, coeffs);

    float energyIn{ 0.0f };
    float energyOut{ 0.0f };
    for (std::size_t i = 0; i < N; ++i)
    {
        energyIn += x[i] * x[i];
        energyOut += coeffs[i] * coeffs[i];
    }

    EXPECT_NEAR(energyIn, energyOut, math::Tolerance<float>());
}

TEST_F(TestDiscreteWaveletTransform, perfect_reconstruction_haar)
{
    Signal16 x;
    for (std::size_t i = 0; i < N; ++i)
        x.push_back(static_cast<float>(i % 7) * 0.3f - 0.5f);

    Signal16 coeffs;
    dwtHaar.Forward(x, coeffs);

    Signal16 xRec;
    dwtHaar.Inverse(coeffs, xRec);

    for (std::size_t i = 0; i < N; ++i)
        EXPECT_NEAR(xRec[i], x[i], math::Tolerance<float>());
}

TEST_F(TestDiscreteWaveletTransform, perfect_reconstruction_daubechies)
{
    Signal16 x;
    for (std::size_t i = 0; i < N; ++i)
        x.push_back(static_cast<float>(i % 5) * 0.4f - 1.0f);

    Signal16 coeffs;
    dwtDb2.Forward(x, coeffs);

    Signal16 xRec;
    dwtDb2.Inverse(coeffs, xRec);

    for (std::size_t i = 0; i < N; ++i)
        EXPECT_NEAR(xRec[i], x[i], math::Tolerance<float>());
}

TEST_F(TestDiscreteWaveletTransform, perfect_reconstruction_sweep_haar)
{
    std::mt19937 rng{ 9876u };
    std::uniform_real_distribution<float> dist{ -5.0f, 5.0f };

    for (int trial = 0; trial < 50; ++trial)
    {
        Signal16 x;
        for (std::size_t i = 0; i < N; ++i)
            x.push_back(dist(rng));

        Signal16 coeffs;
        dwtHaar.Forward(x, coeffs);
        Signal16 xRec;
        dwtHaar.Inverse(coeffs, xRec);

        for (std::size_t i = 0; i < N; ++i)
            EXPECT_NEAR(xRec[i], x[i], math::Tolerance<float>());
    }
}

TEST_F(TestDiscreteWaveletTransform, perfect_reconstruction_sweep_daubechies)
{
    std::mt19937 rng{ 9876u };
    std::uniform_real_distribution<float> dist{ -5.0f, 5.0f };

    for (int trial = 0; trial < 50; ++trial)
    {
        Signal16 x;
        for (std::size_t i = 0; i < N; ++i)
            x.push_back(dist(rng));

        Signal16 coeffs;
        dwtDb2.Forward(x, coeffs);
        Signal16 xRec;
        dwtDb2.Inverse(coeffs, xRec);

        for (std::size_t i = 0; i < N; ++i)
            EXPECT_NEAR(xRec[i], x[i], math::Tolerance<float>());
    }
}

TEST_F(TestDiscreteWaveletTransform, level_offset_matches_analytic_formula)
{
    EXPECT_EQ(dwtHaar.LevelOffset(0), 0u);
    EXPECT_EQ(dwtHaar.LevelOffset(1), N / 2);
    EXPECT_EQ(dwtHaar.LevelOffset(2), N / 2 + N / 4);
    EXPECT_EQ(dwtHaar.LevelOffset(3), N / 2 + N / 4 + N / 8);
}

TEST_F(TestDiscreteWaveletTransform, linearity_holds)
{
    Signal16 x1;
    Signal16 x2;
    for (std::size_t i = 0; i < N; ++i)
    {
        x1.push_back(static_cast<float>(i) * 0.1f);
        x2.push_back(static_cast<float>(N - i) * 0.2f);
    }

    constexpr float a = 2.5f;
    constexpr float b = -1.3f;

    Signal16 xComb;
    for (std::size_t i = 0; i < N; ++i)
        xComb.push_back(a * x1[i] + b * x2[i]);

    Signal16 coeffs1;
    Signal16 coeffs2;
    Signal16 coeffsComb;

    dwtHaar.Forward(x1, coeffs1);
    dwtHaar.Forward(x2, coeffs2);
    dwtHaar.Forward(xComb, coeffsComb);

    for (std::size_t i = 0; i < N; ++i)
        EXPECT_NEAR(coeffsComb[i], a * coeffs1[i] + b * coeffs2[i], math::Tolerance<float>());
}

TEST_F(TestDiscreteWaveletTransform, determinism_same_input_same_output)
{
    Signal16 x;
    for (std::size_t i = 0; i < N; ++i)
        x.push_back(static_cast<float>(i) * 0.7f - 3.0f);

    Signal16 coeffs1;
    Signal16 coeffs2;
    dwtHaar.Forward(x, coeffs1);
    dwtHaar.Forward(x, coeffs2);

    for (std::size_t i = 0; i < N; ++i)
        EXPECT_FLOAT_EQ(coeffs1[i], coeffs2[i]);
}
