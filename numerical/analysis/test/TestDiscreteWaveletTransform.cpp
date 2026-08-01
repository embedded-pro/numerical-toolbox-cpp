#include "numerical/analysis/DiscreteWaveletTransform.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gmock/gmock.h"
#include <cmath>

namespace
{
    class TestDiscreteWaveletTransform : public ::testing::Test
    {
    public:
        static constexpr std::size_t N = 16;

        analysis::WaveletFilters<float, 2> haar{ analysis::MakeHaar<float, 2>() };
        analysis::WaveletFilters<float, 4> db2{ analysis::MakeDaubechies2<float, 4>() };

        analysis::DiscreteWaveletTransform<float, N, 3, 2> dwtHaar{ haar };

        using Signal16 = typename infra::BoundedVector<float>::WithMaxSize<N>;
        using Signal4 = typename infra::BoundedVector<float>::WithMaxSize<4>;
    };
}

TEST_F(TestDiscreteWaveletTransform, haar_of_constant_puts_energy_in_approx)
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
            EXPECT_NEAR(coeffs[offset + i], 0.0f, 1e-5f);
    }

    std::size_t approxOffset = N - (N >> 3);
    float approxEnergy{ 0.0f };
    for (std::size_t i = approxOffset; i < N; ++i)
        approxEnergy += coeffs[i] * coeffs[i];
    EXPECT_GT(approxEnergy, 0.0f);
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

    std::size_t offset0 = dwtHaar.LevelOffset(0);
    float maxDetail{ 0.0f };
    for (std::size_t i = 0; i < N / 2; ++i)
    {
        float v = std::abs(coeffs[offset0 + i]);
        if (v > maxDetail)
            maxDetail = v;
    }
    EXPECT_GT(maxDetail, 1e-3f);
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
        EXPECT_NEAR(xRec[i], x[i], 1e-5f);
}

TEST_F(TestDiscreteWaveletTransform, perfect_reconstruction_daubechies)
{
    analysis::DiscreteWaveletTransform<float, N, 3, 4> dwtDb2{ db2 };

    Signal16 x;
    for (std::size_t i = 0; i < N; ++i)
        x.push_back(static_cast<float>(i % 5) * 0.4f - 1.0f);

    Signal16 coeffs;
    dwtDb2.Forward(x, coeffs);

    Signal16 xRec;
    dwtDb2.Inverse(coeffs, xRec);

    for (std::size_t i = 0; i < N; ++i)
        EXPECT_NEAR(xRec[i], x[i], 1e-4f);
}

TEST_F(TestDiscreteWaveletTransform, energy_is_preserved_orthogonal)
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

    EXPECT_NEAR(energyIn, energyOut, 1e-3f);
}

TEST_F(TestDiscreteWaveletTransform, single_level_matches_manual_qmf)
{
    analysis::DiscreteWaveletTransform<float, 4, 1, 2> dwt1{ haar };

    Signal4 x;
    x.push_back(1.0f);
    x.push_back(2.0f);
    x.push_back(3.0f);
    x.push_back(4.0f);

    Signal4 coeffs;
    dwt1.Forward(x, coeffs);

    constexpr float inv_sqrt2 = 0.7071067811865476f;
    float expCD0 = (1.0f - 2.0f) * inv_sqrt2;
    float expCD1 = (3.0f - 4.0f) * inv_sqrt2;
    float expCA0 = (1.0f + 2.0f) * inv_sqrt2;
    float expCA1 = (3.0f + 4.0f) * inv_sqrt2;

    EXPECT_NEAR(coeffs[0], expCD0, 1e-5f);
    EXPECT_NEAR(coeffs[1], expCD1, 1e-5f);
    EXPECT_NEAR(coeffs[2], expCA0, 1e-5f);
    EXPECT_NEAR(coeffs[3], expCA1, 1e-5f);
}

TEST_F(TestDiscreteWaveletTransform, multilevel_offsets_are_consistent)
{
    std::size_t offset0 = dwtHaar.LevelOffset(0);
    std::size_t offset1 = dwtHaar.LevelOffset(1);
    std::size_t offset2 = dwtHaar.LevelOffset(2);

    EXPECT_EQ(offset0, 0u);
    EXPECT_EQ(offset1, N / 2);
    EXPECT_EQ(offset2, N / 2 + N / 4);

    EXPECT_EQ(offset1 - offset0, N / 2);
    EXPECT_EQ(offset2 - offset1, N / 4);
    EXPECT_EQ(N - offset2, N / 4);
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
        EXPECT_NEAR(coeffsComb[i], a * coeffs1[i] + b * coeffs2[i], 1e-4f);
}
