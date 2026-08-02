#include "numerical/analysis/SignalDetectors.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gmock/gmock.h"
#include <cmath>
#include <numbers>

namespace
{
    class TestSignalDetectors : public ::testing::Test
    {
    public:
        analysis::PeakHold<float> peak{ 1.0f };
        analysis::ZeroCrossingCounter<float> zc{ 0.0f };
        analysis::RmsEnvelope<float> rms{ 0.25f };
    };
}

TEST_F(TestSignalDetectors, peak_hold_latches_maximum)
{
    EXPECT_NEAR(peak.Update(0.2f), 0.2f, math::Tolerance<float>());
    EXPECT_NEAR(peak.Update(0.5f), 0.5f, math::Tolerance<float>());
    EXPECT_NEAR(peak.Update(0.3f), 0.5f, math::Tolerance<float>());
    EXPECT_NEAR(peak.Update(0.1f), 0.5f, math::Tolerance<float>());
}

TEST_F(TestSignalDetectors, peak_hold_decays_when_leaky)
{
    analysis::PeakHold<float> leaky{ 0.5f };
    EXPECT_NEAR(leaky.Update(0.8f), 0.8f, math::Tolerance<float>());
    EXPECT_NEAR(leaky.Update(0.0f), 0.4f, math::Tolerance<float>());
    EXPECT_NEAR(leaky.Update(0.0f), 0.2f, math::Tolerance<float>());
    EXPECT_NEAR(leaky.Update(0.0f), 0.1f, math::Tolerance<float>());
}

TEST_F(TestSignalDetectors, peak_hold_leaky_new_peak_overtakes_decayed)
{
    analysis::PeakHold<float> leaky{ 0.5f };
    leaky.Update(1.0f);
    leaky.Update(0.0f);
    EXPECT_NEAR(leaky.Update(0.9f), 0.9f, math::Tolerance<float>());
}

TEST_F(TestSignalDetectors, peak_hold_tracks_magnitude_not_sign)
{
    EXPECT_NEAR(peak.Update(-0.9f), 0.9f, math::Tolerance<float>());
    EXPECT_NEAR(peak.Update(0.1f), 0.9f, math::Tolerance<float>());
}

TEST_F(TestSignalDetectors, peak_hold_reset_determinism)
{
    peak.Update(1.0f);
    peak.Update(0.5f);
    peak.Reset();

    analysis::PeakHold<float> fresh{ 1.0f };
    EXPECT_FLOAT_EQ(peak.Update(0.3f), fresh.Update(0.3f));
    EXPECT_FLOAT_EQ(peak.Update(0.7f), fresh.Update(0.7f));
}

TEST_F(TestSignalDetectors, zero_crossing_counts_sign_changes)
{
    zc.Update(1.0f);
    zc.Update(-1.0f);
    zc.Update(1.0f);
    zc.Update(-1.0f);
    EXPECT_EQ(zc.Count(), 3u);
}

TEST_F(TestSignalDetectors, zero_crossing_update_returns_true_on_cross)
{
    zc.Update(1.0f);
    EXPECT_TRUE(zc.Update(-1.0f));
}

TEST_F(TestSignalDetectors, zero_crossing_update_returns_false_when_no_cross)
{
    zc.Update(1.0f);
    EXPECT_FALSE(zc.Update(0.5f));
}

TEST_F(TestSignalDetectors, zero_crossing_hysteresis_rejects_noise)
{
    analysis::ZeroCrossingCounter<float> noisy{ 0.1f };
    noisy.Update(0.02f);
    noisy.Update(-0.02f);
    noisy.Update(0.02f);
    EXPECT_EQ(noisy.Count(), 0u);
}

TEST_F(TestSignalDetectors, zero_crossing_hysteresis_passes_above_threshold)
{
    analysis::ZeroCrossingCounter<float> filtered{ 0.1f };
    filtered.Update(0.5f);
    filtered.Update(-0.5f);
    filtered.Update(0.5f);
    EXPECT_EQ(filtered.Count(), 2u);
}

TEST_F(TestSignalDetectors, rms_envelope_converges_to_true_rms)
{
    analysis::RmsEnvelope<float> r{ 0.1f };
    constexpr int fs{ 48000 };
    constexpr int freq{ 1000 };
    constexpr int period{ fs / freq };
    constexpr int warmup{ 5000 };
    constexpr float amplitude{ 0.5f * 1.41421356f };
    for (int i = 0; i < warmup; ++i)
        r.Update(amplitude * std::sin(std::numbers::pi_v<float> * 2.0f * static_cast<float>(freq) * static_cast<float>(i) / static_cast<float>(fs)));
    float sum{ 0.0f };
    for (int i = 0; i < period; ++i)
        sum += r.Update(amplitude * std::sin(std::numbers::pi_v<float> * 2.0f * static_cast<float>(freq) * static_cast<float>(warmup + i) / static_cast<float>(fs)));
    EXPECT_NEAR(sum / static_cast<float>(period), 0.5f, 1e-2f);
}

TEST_F(TestSignalDetectors, rms_dc_input_returns_magnitude)
{
    analysis::RmsEnvelope<float> r{ 0.25f };
    float result{ 0.0f };
    for (int i = 0; i < 200; ++i)
        result = r.Update(0.4f);
    EXPECT_NEAR(result, 0.4f, 1e-2f);
}

TEST_F(TestSignalDetectors, rms_envelope_reset_nonzero_seeds_state)
{
    constexpr float seedValue{ 0.09f };
    rms.Reset(seedValue);
    EXPECT_NEAR(rms.Update(std::sqrt(seedValue)), std::sqrt(seedValue), math::Tolerance<float>());
}

TEST_F(TestSignalDetectors, reset_clears_all_state)
{
    peak.Update(1.0f);
    zc.Update(1.0f);
    zc.Update(-1.0f);
    rms.Update(1.0f);

    peak.Reset();
    zc.Reset();
    rms.Reset();

    EXPECT_NEAR(peak.Update(0.0f), 0.0f, math::Tolerance<float>());
    EXPECT_EQ(zc.Count(), 0u);
    EXPECT_NEAR(rms.Update(0.0f), 0.0f, math::Tolerance<float>());
}
