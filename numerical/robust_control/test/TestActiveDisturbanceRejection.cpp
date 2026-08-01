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

    class TestActiveDisturbanceRejection : public ::testing::Test
    {
    protected:
        robust_control::ActiveDisturbanceRejectionControl<float, 2> adrc{ kWo, kWc, kB0, kTs };
        SecondOrderPlant plant{ kB0 };
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
