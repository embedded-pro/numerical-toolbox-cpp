#include "numerical/filters/passive/BiquadCascade.hpp"
#include "numerical/filters/passive/IirFilterDesign.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gtest/gtest.h"
#include <array>
#include <cmath>
#include <numbers>

namespace
{
    class TestIirFilterDesign
        : public ::testing::Test
    {
    public:
        filters::passive::IirFilterDesign<float, 8> designer{};
    };

    float MeasureMagnitude(filters::passive::IirFilterDesign<float, 8>& designer, float freqHz, float sampleHz, std::size_t numSections)
    {
        std::array<filters::passive::BiquadCoeffs<float>, 4> coeffs{};
        for (std::size_t i{ 0 }; i < numSections && i < 4; ++i)
            coeffs[i] = designer.Section(i);

        const float w{ 2.0f * std::numbers::pi_v<float> * freqHz / sampleHz };
        constexpr int kSettle{ 2000 };
        constexpr int kMeasure{ 500 };

        std::array<float, 4> z1{};
        std::array<float, 4> z2{};

        auto filterSample = [&](float x) -> float
        {
            for (std::size_t s{ 0 }; s < numSections && s < 4; ++s)
            {
                const auto& c{ coeffs[s] };
                const float y{ c.b0 * x + z1[s] };
                z1[s] = c.b1 * x - c.a1 * y + z2[s];
                z2[s] = c.b2 * x - c.a2 * y;
                x = y;
            }
            return x;
        };

        for (int i{ 0 }; i < kSettle; ++i)
            filterSample(std::sin(static_cast<float>(i) * w));

        float maxAmp{ 0.0f };
        for (int i{ kSettle }; i < kSettle + kMeasure; ++i)
        {
            const float y{ filterSample(std::sin(static_cast<float>(i) * w)) };
            const float absY{ y < 0.0f ? -y : y };
            if (absY > maxAmp)
                maxAmp = absY;
        }
        return maxAmp;
    }
}

TEST_F(TestIirFilterDesign, butterworth_lp_section_count)
{
    const std::size_t n{ designer.Design(filters::passive::Prototype::Butterworth, filters::passive::Kind::LowPass, 4, 100.0f, 1000.0f) };
    EXPECT_EQ(n, 2u);
}

TEST_F(TestIirFilterDesign, butterworth_lp_dc_gain_unity)
{
    const std::size_t n{ designer.Design(filters::passive::Prototype::Butterworth, filters::passive::Kind::LowPass, 2, 100.0f, 1000.0f) };
    ASSERT_EQ(n, 1u);

    const float dcMag{ MeasureMagnitude(designer, 0.5f, 1000.0f, n) };
    EXPECT_NEAR(dcMag, 1.0f, 5e-2f);
}

TEST_F(TestIirFilterDesign, cutoff_is_minus_3db)
{
    constexpr float fc{ 100.0f };
    constexpr float fs{ 1000.0f };
    const std::size_t n{ designer.Design(filters::passive::Prototype::Butterworth, filters::passive::Kind::LowPass, 2, fc, fs) };
    ASSERT_GT(n, 0u);

    const float mag{ MeasureMagnitude(designer, fc, fs, n) };
    EXPECT_NEAR(mag, 1.0f / std::sqrt(2.0f), 5e-2f);
}

TEST_F(TestIirFilterDesign, prewarp_places_cutoff_exactly)
{
    constexpr float fs{ 1000.0f };
    constexpr std::array<float, 3> testFc{ 50.0f, 100.0f, 200.0f };

    for (const float fc : testFc)
    {
        const std::size_t n{ designer.Design(filters::passive::Prototype::Butterworth, filters::passive::Kind::LowPass, 2, fc, fs) };
        ASSERT_GT(n, 0u);

        const float mag{ MeasureMagnitude(designer, fc, fs, n) };
        EXPECT_NEAR(mag, 1.0f / std::sqrt(2.0f), 0.1f);
    }
}

TEST_F(TestIirFilterDesign, chebyshev_has_passband_ripple)
{
    constexpr float fc{ 100.0f };
    constexpr float fs{ 1000.0f };
    constexpr float rippleDb{ 1.0f };

    filters::passive::IirFilterDesign<float, 8> butterDesigner{};
    const std::size_t nb{ butterDesigner.Design(filters::passive::Prototype::Butterworth, filters::passive::Kind::LowPass, 4, fc, fs) };
    const std::size_t nc{ designer.Design(filters::passive::Prototype::ChebyshevI, filters::passive::Kind::LowPass, 4, fc, fs, rippleDb) };

    ASSERT_GT(nc, 0u);
    ASSERT_GT(nb, 0u);

    const float stopFreq{ 300.0f };
    const float chebyStop{ MeasureMagnitude(designer, stopFreq, fs, nc) };
    const float butterStop{ MeasureMagnitude(butterDesigner, stopFreq, fs, nb) };

    EXPECT_LT(chebyStop, butterStop);
}

TEST_F(TestIirFilterDesign, highpass_blocks_dc)
{
    constexpr float fs{ 1000.0f };
    const std::size_t n{ designer.Design(filters::passive::Prototype::Butterworth, filters::passive::Kind::HighPass, 2, 100.0f, fs) };
    ASSERT_GT(n, 0u);

    const float dcMag{ MeasureMagnitude(designer, 0.5f, fs, n) };
    EXPECT_LT(dcMag, 0.05f);
}

TEST_F(TestIirFilterDesign, butterworth_lp_odd_order_dc_gain_unity)
{
    constexpr float fs{ 1000.0f };
    const std::size_t n{ designer.Design(filters::passive::Prototype::Butterworth, filters::passive::Kind::LowPass, 3, 100.0f, fs) };
    ASSERT_EQ(n, 2u);

    const float dcMag{ MeasureMagnitude(designer, 0.5f, fs, n) };
    EXPECT_NEAR(dcMag, 1.0f, 5e-2f);
}

TEST_F(TestIirFilterDesign, bandpass_passes_center_over_edges)
{
    constexpr float fs{ 1000.0f };
    constexpr float fc{ 100.0f };
    const std::size_t n{ designer.Design(filters::passive::Prototype::Butterworth, filters::passive::Kind::BandPass, 2, fc, fs) };
    ASSERT_GT(n, 0u);

    const float centerMag{ MeasureMagnitude(designer, fc, fs, n) };
    const float dcMag{ MeasureMagnitude(designer, 0.5f, fs, n) };
    const float highMag{ MeasureMagnitude(designer, 450.0f, fs, n) };

    EXPECT_GT(centerMag, dcMag);
    EXPECT_GT(centerMag, highMag);
}

TEST_F(TestIirFilterDesign, bandstop_attenuates_center)
{
    constexpr float fs{ 1000.0f };
    constexpr float fc{ 100.0f };
    const std::size_t n{ designer.Design(filters::passive::Prototype::Butterworth, filters::passive::Kind::BandStop, 2, fc, fs) };
    ASSERT_GT(n, 0u);

    const float notchMag{ MeasureMagnitude(designer, fc, fs, n) };
    const float passMag{ MeasureMagnitude(designer, 0.5f, fs, n) };

    EXPECT_LT(notchMag, passMag);
}

TEST_F(TestIirFilterDesign, section_out_of_range_is_zeroed)
{
    const std::size_t n{ designer.Design(filters::passive::Prototype::Butterworth, filters::passive::Kind::LowPass, 2, 100.0f, 1000.0f) };
    ASSERT_EQ(n, 1u);

    const auto c{ designer.Section(n) };
    EXPECT_EQ(c.b0, 0.0f);
    EXPECT_EQ(c.b1, 0.0f);
    EXPECT_EQ(c.b2, 0.0f);
    EXPECT_EQ(c.a1, 0.0f);
    EXPECT_EQ(c.a2, 0.0f);
}

TEST_F(TestIirFilterDesign, design_is_stable)
{
    constexpr float fs{ 1000.0f };
    constexpr std::array<float, 3> cutoffs{ 50.0f, 100.0f, 200.0f };
    constexpr std::array<std::size_t, 3> orders{ 2u, 4u, 6u };

    for (const float fc : cutoffs)
    {
        for (const std::size_t ord : orders)
        {
            if (ord > 8)
                continue;
            const std::size_t n{ designer.Design(filters::passive::Prototype::Butterworth, filters::passive::Kind::LowPass, ord, fc, fs) };
            ASSERT_GT(n, 0u);

            for (std::size_t s{ 0 }; s < n; ++s)
            {
                const auto c{ designer.Section(s) };
                if (c.a2 > 0.0f)
                {
                    EXPECT_LT(c.a2, 1.0f);
                    EXPECT_GT(c.a2, -1.0f);
                }
            }
        }
    }
}
