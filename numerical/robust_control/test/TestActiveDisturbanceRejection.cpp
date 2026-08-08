#include "numerical/math/Tolerance.hpp"
#include "numerical/robust_control/ActiveDisturbanceRejection.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    static constexpr float kWo{ 30.0f };
    static constexpr float kWc{ 6.0f };
    static constexpr float kB0{ 1.0f };
    static constexpr float kTs{ 0.001f };

    struct SecondOrderPlant
    {
        float y{ 0.0f };
        float ydot{ 0.0f };
        float bTrue;

        explicit SecondOrderPlant(float b)
            : bTrue{ b }
        {}

        float Step(float u, float disturbance = 0.0f)
        {
            const float yddot = bTrue * u + disturbance;
            ydot += kTs * yddot;
            y += kTs * ydot;
            return y;
        }
    };

    struct FirstOrderPlant
    {
        float y{ 0.0f };
        float bTrue;

        explicit FirstOrderPlant(float b)
            : bTrue{ b }
        {}

        float Step(float u, float disturbance = 0.0f)
        {
            y += kTs * (bTrue * u + disturbance);
            return y;
        }
    };

    class TestActiveDisturbanceRejection : public ::testing::Test
    {
    protected:
        robust_control::ActiveDisturbanceRejectionControl<float, 2> adrc{ kWo, kWc, kB0, kTs };
        SecondOrderPlant plant{ kB0 };
    };

    class TestActiveDisturbanceRejectionOrder1 : public ::testing::Test
    {
    protected:
        static constexpr float kWo1{ 20.0f };
        static constexpr float kWc1{ 5.0f };
        robust_control::ActiveDisturbanceRejectionControl<float, 1> adrc{ kWo1, kWc1, kB0, kTs };
        FirstOrderPlant plant{ kB0 };
    };
}

TEST_F(TestActiveDisturbanceRejection, bandwidth_gain_mapping)
{
    const auto og = robust_control::ActiveDisturbanceRejectionControl<float, 2>::ObserverGainFromBandwidth(kWo);
    const auto cg = robust_control::ActiveDisturbanceRejectionControl<float, 2>::ControlGainFromBandwidth(kWc);

    EXPECT_NEAR(og.at(0, 0), 3.0f * kWo, math::Tolerance<float>());
    EXPECT_NEAR(og.at(1, 0), 3.0f * kWo * kWo, math::Tolerance<float>());
    EXPECT_NEAR(og.at(2, 0), kWo * kWo * kWo, math::Tolerance<float>());

    EXPECT_NEAR(cg.at(0, 0), kWc * kWc, math::Tolerance<float>());
    EXPECT_NEAR(cg.at(1, 0), 2.0f * kWc, math::Tolerance<float>());
}

TEST_F(TestActiveDisturbanceRejection, reset_clears_observer)
{
    for (int i = 0; i < 50; ++i)
        adrc.Compute(1.0f, plant.Step(0.1f));

    adrc.Reset();

    const auto& xhat = adrc.EstimatedState();
    for (std::size_t i = 0; i < 3; ++i)
        EXPECT_NEAR(xhat.at(i, 0), 0.0f, math::Tolerance<float>());

    EXPECT_NEAR(adrc.AppliedPrev(), 0.0f, math::Tolerance<float>());
}

TEST_F(TestActiveDisturbanceRejection, eso_converges)
{
    for (int i = 0; i < 3000; ++i)
    {
        const float y = plant.Step(adrc.Compute(0.0f, plant.y));
        (void)y;
    }

    const float estimationError = plant.y - adrc.EstimatedState().at(0, 0);
    EXPECT_NEAR(estimationError, 0.0f, 1e-2f);
}

TEST_F(TestActiveDisturbanceRejection, tracks_step_reference)
{
    const float reference{ 1.0f };

    for (int i = 0; i < 5000; ++i)
        plant.Step(adrc.Compute(reference, plant.y));

    EXPECT_NEAR(plant.y, reference, 1e-2f);
}

TEST_F(TestActiveDisturbanceRejection, estimates_total_disturbance)
{
    const float constantDisturbance{ 5.0f };

    for (int i = 0; i < 5000; ++i)
        plant.Step(adrc.Compute(0.0f, plant.y), constantDisturbance);

    EXPECT_NEAR(adrc.EstimatedState().at(2, 0), constantDisturbance, 1.0f);
}

TEST_F(TestActiveDisturbanceRejection, rejects_step_disturbance)
{
    const float reference{ 1.0f };

    for (int i = 0; i < 3000; ++i)
        plant.Step(adrc.Compute(reference, plant.y));

    const float disturbance{ 3.0f };
    for (int i = 0; i < 5000; ++i)
        plant.Step(adrc.Compute(reference, plant.y), disturbance);

    EXPECT_NEAR(plant.y, reference, 5e-2f);
}

TEST_F(TestActiveDisturbanceRejection, control_cancels_disturbance_term)
{
    const float reference{ 0.0f };
    const float injectedDisturbance{ 10.0f };

    for (int i = 0; i < 4000; ++i)
        plant.Step(adrc.Compute(reference, plant.y), injectedDisturbance);

    const float fhat = adrc.EstimatedState().at(2, 0);
    const float u = adrc.Compute(reference, plant.y);

    EXPECT_LT(fhat * u, 0.0f);
}

TEST_F(TestActiveDisturbanceRejection, near_model_free_robustness)
{
    const float bTrue{ 2.0f };
    SecondOrderPlant mismatchedPlant{ bTrue };
    robust_control::ActiveDisturbanceRejectionControl<float, 2> controller{ kWo, kWc, kB0, kTs };

    const float reference{ 1.0f };
    for (int i = 0; i < 8000; ++i)
        mismatchedPlant.Step(controller.Compute(reference, mismatchedPlant.y));

    EXPECT_NEAR(mismatchedPlant.y, reference, 0.1f);
}

TEST_F(TestActiveDisturbanceRejectionOrder1, bandwidth_gain_mapping_order1)
{
    const auto og = robust_control::ActiveDisturbanceRejectionControl<float, 1>::ObserverGainFromBandwidth(kWo1);
    const auto cg = robust_control::ActiveDisturbanceRejectionControl<float, 1>::ControlGainFromBandwidth(kWc1);

    EXPECT_NEAR(og.at(0, 0), 2.0f * kWo1, math::Tolerance<float>());
    EXPECT_NEAR(og.at(1, 0), 1.0f * kWo1 * kWo1, math::Tolerance<float>());

    EXPECT_NEAR(cg.at(0, 0), 1.0f * kWc1, math::Tolerance<float>());
}

TEST_F(TestActiveDisturbanceRejection, zero_input_zero_output_first_call)
{
    const float u = adrc.Compute(0.0f, 0.0f);
    EXPECT_NEAR(u, 0.0f, math::Tolerance<float>());
}

TEST_F(TestActiveDisturbanceRejection, long_horizon_bounded_output)
{
    const float reference{ 1.0f };
    float maxOutput{ 0.0f };

    for (int i = 0; i < 20000; ++i)
    {
        const float u = adrc.Compute(reference, plant.y);
        plant.Step(u);
        const float absY = std::abs(plant.y);
        if (absY > maxOutput)
            maxOutput = absY;
    }

    EXPECT_LT(maxOutput, 10.0f);
    EXPECT_FALSE(std::isnan(plant.y));
    EXPECT_FALSE(std::isinf(plant.y));
}

TEST_F(TestActiveDisturbanceRejection, reset_mid_run_matches_fresh_instance)
{
    const float reference{ 1.0f };

    for (int i = 0; i < 500; ++i)
        plant.Step(adrc.Compute(reference, plant.y));

    adrc.Reset();
    plant = SecondOrderPlant{ kB0 };

    robust_control::ActiveDisturbanceRejectionControl<float, 2> fresh{ kWo, kWc, kB0, kTs };
    SecondOrderPlant freshPlant{ kB0 };

    for (int i = 0; i < 200; ++i)
    {
        const float uReset = adrc.Compute(reference, plant.y);
        const float uFresh = fresh.Compute(reference, freshPlant.y);
        plant.Step(uReset);
        freshPlant.Step(uFresh);
    }

    EXPECT_FLOAT_EQ(plant.y, freshPlant.y);
    EXPECT_FLOAT_EQ(adrc.AppliedPrev(), fresh.AppliedPrev());
}
