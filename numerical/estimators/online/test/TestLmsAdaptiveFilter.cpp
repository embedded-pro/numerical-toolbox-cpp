#include "numerical/estimators/online/LmsAdaptiveFilter.hpp"
#include "numerical/math/Tolerance.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    class TestLmsAdaptiveFilter
        : public ::testing::Test
    {
    protected:
        estimators::LmsAdaptiveFilter<float, 4> lms{ 0.1f };
    };
}

TEST_F(TestLmsAdaptiveFilter, identifies_static_fir_system)
{
    constexpr std::array<float, 4> plant{ 0.5f, -0.25f, 0.125f, -0.0625f };

    std::array<float, 4> inputHistory{};
    uint32_t seed{ 42 };
    auto nextSample = [&seed]() -> float
    {
        seed = seed * 1664525u + 1013904223u;
        return static_cast<float>(static_cast<int32_t>(seed)) / static_cast<float>(1u << 31u);
    };

    for (int i{ 0 }; i < 4000; ++i)
    {
        float input{ nextSample() };
        for (int j{ 3 }; j > 0; --j)
            inputHistory[j] = inputHistory[j - 1];
        inputHistory[0] = input;

        float desired{ 0.0f };
        for (std::size_t k{ 0 }; k < 4; ++k)
            desired += plant[k] * inputHistory[k];

        lms.Update(input, desired);
    }

    const auto& w{ lms.Weights() };
    EXPECT_NEAR(w[0], plant[0], 0.01f);
    EXPECT_NEAR(w[1], plant[1], 0.01f);
    EXPECT_NEAR(w[2], plant[2], 0.01f);
    EXPECT_NEAR(w[3], plant[3], 0.01f);
}

TEST_F(TestLmsAdaptiveFilter, error_decreases_on_average)
{
    constexpr std::array<float, 4> plant{ 0.5f, 0.5f, 0.0f, 0.0f };

    std::array<float, 4> inputHistory{};
    uint32_t seed{ 99 };
    auto nextSample = [&seed]() -> float
    {
        seed = seed * 1664525u + 1013904223u;
        return static_cast<float>(static_cast<int32_t>(seed)) / static_cast<float>(1u << 31u);
    };

    float earlyMse{ 0.0f };
    float lateMse{ 0.0f };
    constexpr int total{ 2000 };
    constexpr int half{ total / 2 };

    for (int i{ 0 }; i < total; ++i)
    {
        float input{ nextSample() };
        for (int j{ 3 }; j > 0; --j)
            inputHistory[j] = inputHistory[j - 1];
        inputHistory[0] = input;

        float desired{ 0.0f };
        for (std::size_t k{ 0 }; k < 4; ++k)
            desired += plant[k] * inputHistory[k];

        auto result{ lms.Update(input, desired) };

        if (i < half)
            earlyMse += result.error * result.error;
        else
            lateMse += result.error * result.error;
    }

    EXPECT_LT(lateMse, earlyMse);
}

TEST_F(TestLmsAdaptiveFilter, zero_error_freezes_weights)
{
    lms.Update(1.0f, 0.0f);
    lms.Update(1.0f, 0.0f);

    const std::array<float, 4> weightsBefore{ lms.Weights() };

    float output{ lms.Update(1.0f, 0.0f).output };
    lms.Update(1.0f, output);

    const auto& weightsAfter{ lms.Weights() };
    for (std::size_t i{ 0 }; i < 4; ++i)
        EXPECT_NEAR(weightsAfter[i], weightsBefore[i], math::Tolerance<float>());
}

TEST_F(TestLmsAdaptiveFilter, nlms_converges_independent_of_input_scale)
{
    constexpr std::array<float, 4> plant{ 0.5f, 0.5f, 0.0f, 0.0f };

    estimators::LmsAdaptiveFilter<float, 4> nlmsUnscaled{ 0.5f, true };
    estimators::LmsAdaptiveFilter<float, 4> nlmsScaled{ 0.5f, true };

    std::array<float, 4> histNorm{};
    std::array<float, 4> histScaled{};

    uint32_t seed{ 77 };
    auto nextSample = [&seed]() -> float
    {
        seed = seed * 1664525u + 1013904223u;
        return static_cast<float>(static_cast<int32_t>(seed)) / static_cast<float>(1u << 31u);
    };

    constexpr int iterations{ 2000 };
    float errNorm{ 0.0f };
    float errScaled{ 0.0f };

    for (int i{ 0 }; i < iterations; ++i)
    {
        float base{ nextSample() };
        float scaled{ base * 10.0f };

        for (int j{ 3 }; j > 0; --j)
        {
            histNorm[j] = histNorm[j - 1];
            histScaled[j] = histScaled[j - 1];
        }
        histNorm[0] = base;
        histScaled[0] = scaled;

        float desiredNorm{ 0.0f };
        float desiredScaled{ 0.0f };
        for (std::size_t k{ 0 }; k < 4; ++k)
        {
            desiredNorm += plant[k] * histNorm[k];
            desiredScaled += plant[k] * histScaled[k];
        }

        auto rn{ nlmsUnscaled.Update(base, desiredNorm) };
        auto rs{ nlmsScaled.Update(scaled, desiredScaled) };

        if (i >= iterations / 2)
        {
            errNorm += rn.error * rn.error;
            errScaled += rs.error * rs.error;
        }
    }

    const auto& wn{ nlmsUnscaled.Weights() };
    const auto& ws{ nlmsScaled.Weights() };
    EXPECT_NEAR(wn[0], plant[0], 0.05f);
    EXPECT_NEAR(ws[0], plant[0], 0.05f);
}

TEST_F(TestLmsAdaptiveFilter, larger_mu_converges_faster)
{
    constexpr std::array<float, 4> plant{ 0.5f, 0.5f, 0.0f, 0.0f };

    estimators::LmsAdaptiveFilter<float, 4> slowFilter{ 0.05f };
    estimators::LmsAdaptiveFilter<float, 4> fastFilter{ 0.2f };

    std::array<float, 4> histSlow{};
    std::array<float, 4> histFast{};

    uint32_t seed{ 123 };
    auto nextSample = [&seed]() -> float
    {
        seed = seed * 1664525u + 1013904223u;
        return static_cast<float>(static_cast<int32_t>(seed)) / static_cast<float>(1u << 31u);
    };

    constexpr float threshold{ 0.01f };
    int slowConverged{ -1 };
    int fastConverged{ -1 };
    constexpr int maxIter{ 5000 };

    for (int i{ 0 }; i < maxIter; ++i)
    {
        float input{ nextSample() };
        for (int j{ 3 }; j > 0; --j)
        {
            histSlow[j] = histSlow[j - 1];
            histFast[j] = histFast[j - 1];
        }
        histSlow[0] = input;
        histFast[0] = input;

        float desired{ 0.0f };
        for (std::size_t k{ 0 }; k < 4; ++k)
            desired += plant[k] * histSlow[k];

        auto rs{ slowFilter.Update(input, desired) };
        auto rf{ fastFilter.Update(input, desired) };

        if (slowConverged < 0 && std::abs(rs.error) < threshold)
            slowConverged = i;
        if (fastConverged < 0 && std::abs(rf.error) < threshold)
            fastConverged = i;
    }

    EXPECT_LT(fastConverged, slowConverged);
}

TEST_F(TestLmsAdaptiveFilter, reset_clears_weights_and_history)
{
    constexpr std::array<float, 4> plant{ 0.5f, 0.5f, 0.0f, 0.0f };
    std::array<float, 4> hist{};

    uint32_t seed{ 11 };
    auto nextSample = [&seed]() -> float
    {
        seed = seed * 1664525u + 1013904223u;
        return static_cast<float>(static_cast<int32_t>(seed)) / static_cast<float>(1u << 31u);
    };

    for (int i{ 0 }; i < 500; ++i)
    {
        float input{ nextSample() };
        for (int j{ 3 }; j > 0; --j)
            hist[j] = hist[j - 1];
        hist[0] = input;

        float desired{ 0.0f };
        for (std::size_t k{ 0 }; k < 4; ++k)
            desired += plant[k] * hist[k];

        lms.Update(input, desired);
    }

    lms.Reset();

    const auto& w{ lms.Weights() };
    for (std::size_t i{ 0 }; i < 4; ++i)
        EXPECT_NEAR(w[i], 0.0f, math::Tolerance<float>());

    float out{ lms.Update(1.0f, 0.0f).output };
    EXPECT_NEAR(out, 0.0f, math::Tolerance<float>());
}

TEST_F(TestLmsAdaptiveFilter, output_equals_fir_of_weights)
{
    estimators::LmsAdaptiveFilter<float, 4> filter{ 0.0f };

    filter.Update(0.0f, 0.0f);
    filter.Update(0.0f, 0.0f);
    filter.Update(0.0f, 0.0f);
    filter.Update(0.0f, 0.0f);

    const std::array<float, 4> knownWeights{ 0.3f, 0.6f, 0.1f, -0.2f };
    auto& w{ const_cast<std::array<float, 4>&>(filter.Weights()) };
    w = knownWeights;

    float out0{ filter.Update(1.0f, 0.0f).output };
    float out1{ filter.Update(0.0f, 0.0f).output };
    float out2{ filter.Update(0.0f, 0.0f).output };
    float out3{ filter.Update(0.0f, 0.0f).output };

    EXPECT_NEAR(out0, knownWeights[0], math::Tolerance<float>());
    EXPECT_NEAR(out1, knownWeights[1], math::Tolerance<float>());
    EXPECT_NEAR(out2, knownWeights[2], math::Tolerance<float>());
    EXPECT_NEAR(out3, knownWeights[3], math::Tolerance<float>());
}
