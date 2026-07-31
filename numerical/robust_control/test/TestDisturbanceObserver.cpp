// Copyright (c) 2024, Numerical Toolbox Contributors. All rights reserved.
#include "numerical/filters/passive/BiquadCascade.hpp"
#include "numerical/math/LinearTimeInvariant.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/Tolerance.hpp"
#include "numerical/robust_control/DisturbanceObserver.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    static constexpr float kCutoffHz{ 20.0f };
    static constexpr float kSampleRateHz{ 1000.0f };
    static constexpr float kQ{ 0.7071f };

    static constexpr float kPlantA{ 0.9f };
    static constexpr float kPlantB{ 0.1f };
    static constexpr float kPlantC{ 1.0f };

    math::LinearTimeInvariant<float, 1, 1, 1> MakeFirstOrderPlant()
    {
        math::LinearTimeInvariant<float, 1, 1, 1> plant{};
        plant.A.at(0, 0) = kPlantA;
        plant.B.at(0, 0) = kPlantB;
        plant.C.at(0, 0) = kPlantC;
        return plant;
    }

    filters::passive::BiquadCoeffs<float> MakeLowPassQ()
    {
        return filters::passive::Biquad<float>::LowPass(kCutoffHz, kSampleRateHz, kQ);
    }

    class TestDisturbanceObserver : public ::testing::Test
    {
    protected:
        math::LinearTimeInvariant<float, 1, 1, 1> nominal{ MakeFirstOrderPlant() };
        filters::passive::BiquadCoeffs<float> qCoeffs{ MakeLowPassQ() };
        robust_control::DisturbanceObserver<float, 1, 1, 1> dob{ nominal, qCoeffs };
    };
}

TEST_F(TestDisturbanceObserver, estimates_constant_disturbance)
{
    static constexpr float kDisturbance{ 0.5f };
    static constexpr int kSteps{ 4000 };

    math::Vector<float, 1> x{};
    math::Vector<float, 1> c{};
    math::Vector<float, 1> u{};
    math::Vector<float, 1> y{};

    for (int k{ 0 }; k < kSteps; ++k)
    {
        y = nominal.Output(x, u);
        u = dob.Compute(c, y);
        math::Vector<float, 1> uActual{};
        uActual.at(0, 0) = u.at(0, 0) + kDisturbance;
        x = nominal.Step(x, uActual);
    }

    EXPECT_NEAR(dob.Disturbance().at(0, 0), kDisturbance, 1e-2f);
}

TEST_F(TestDisturbanceObserver, cancels_disturbance_at_output)
{
    static constexpr float kDisturbance{ 0.3f };
    static constexpr int kSteps{ 4000 };

    math::LinearTimeInvariant<float, 1, 1, 1> nominalRef{ MakeFirstOrderPlant() };

    math::Vector<float, 1> xDob{};
    math::Vector<float, 1> xNominal{};
    math::Vector<float, 1> c{};
    math::Vector<float, 1> uDob{};
    math::Vector<float, 1> uNominal{};
    math::Vector<float, 1> yDob{};
    math::Vector<float, 1> yNominal{};

    for (int k{ 0 }; k < kSteps; ++k)
    {
        yDob = nominal.Output(xDob, uDob);
        yNominal = nominalRef.Output(xNominal, uNominal);

        uDob = dob.Compute(c, yDob);
        uNominal = c;

        math::Vector<float, 1> uActual{};
        uActual.at(0, 0) = uDob.at(0, 0) + kDisturbance;
        xDob = nominal.Step(xDob, uActual);
        xNominal = nominalRef.Step(xNominal, uNominal);
    }

    EXPECT_NEAR(yDob.at(0, 0), yNominal.at(0, 0), 1e-2f);
}

TEST_F(TestDisturbanceObserver, transparent_outside_q_bandwidth)
{
    static constexpr float kHighFreqDistAmp{ 1.0f };
    static constexpr float kHighFreqHz{ 400.0f };
    static constexpr int kSteps{ 500 };

    math::Vector<float, 1> x{};
    math::Vector<float, 1> c{};
    math::Vector<float, 1> u{};
    math::Vector<float, 1> y{};

    float sumDistSq{ 0.0f };

    for (int k{ 0 }; k < kSteps; ++k)
    {
        const float d{ kHighFreqDistAmp *
            std::sin(2.0f * std::numbers::pi_v<float> * kHighFreqHz * static_cast<float>(k) / kSampleRateHz) };
        y = nominal.Output(x, u);
        u = dob.Compute(c, y);
        math::Vector<float, 1> uActual{};
        uActual.at(0, 0) = u.at(0, 0) + d;
        x = nominal.Step(x, uActual);
        if (k > 100)
            sumDistSq += dob.Disturbance().at(0, 0) * dob.Disturbance().at(0, 0);
    }

    const float rmsEst{ std::sqrt(sumDistSq / static_cast<float>(kSteps - 100)) };
    EXPECT_LT(rmsEst, 0.1f);
}

TEST_F(TestDisturbanceObserver, unity_dc_gain_q_filter)
{
    static constexpr float kConstInput{ 1.0f };
    static constexpr int kSteps{ 4000 };

    filters::passive::BiquadCascade<float, 1> qFilter{ { qCoeffs } };
    float output{ 0.0f };

    for (int k{ 0 }; k < kSteps; ++k)
        output = qFilter.Filter(kConstInput);

    EXPECT_NEAR(output, kConstInput, 1e-2f);
}

TEST_F(TestDisturbanceObserver, recovers_nominal_plant_behaviour)
{
    static constexpr float kGainPerturbation{ 1.05f };
    static constexpr int kSteps{ 4000 };

    math::LinearTimeInvariant<float, 1, 1, 1> perturbedPlant{ MakeFirstOrderPlant() };
    perturbedPlant.B.at(0, 0) *= kGainPerturbation;
    perturbedPlant.C.at(0, 0) *= kGainPerturbation;

    math::LinearTimeInvariant<float, 1, 1, 1> refPlant{ MakeFirstOrderPlant() };

    math::Vector<float, 1> xDob{};
    math::Vector<float, 1> xRef{};
    math::Vector<float, 1> uDob{};
    math::Vector<float, 1> uRef{};
    math::Vector<float, 1> yDob{};
    math::Vector<float, 1> yRef{};

    const math::Vector<float, 1> c{ { 1.0f } };

    for (int k{ 0 }; k < kSteps; ++k)
    {
        yDob = perturbedPlant.Output(xDob, uDob);
        yRef = refPlant.Output(xRef, uRef);

        uDob = dob.Compute(c, yDob);
        uRef = c;

        xDob = perturbedPlant.Step(xDob, uDob);
        xRef = refPlant.Step(xRef, uRef);
    }

    EXPECT_NEAR(yDob.at(0, 0), yRef.at(0, 0), 5e-2f);
}

TEST_F(TestDisturbanceObserver, proper_realization_no_differentiation)
{
    static constexpr float kNoiseAmp{ 1.0f };
    static constexpr int kSteps{ 200 };
    static constexpr float kBoundedThreshold{ 10.0f };

    math::Vector<float, 1> c{};
    math::Vector<float, 1> y{};

    for (int k{ 0 }; k < kSteps; ++k)
    {
        y.at(0, 0) = (k % 2 == 0) ? kNoiseAmp : -kNoiseAmp;
        dob.Compute(c, y);
        EXPECT_LT(std::abs(dob.Disturbance().at(0, 0)), kBoundedThreshold);
    }
}

TEST_F(TestDisturbanceObserver, reset_clears_estimate)
{
    static constexpr float kDisturbance{ 1.0f };
    static constexpr int kSteps{ 2000 };

    math::Vector<float, 1> x{};
    math::Vector<float, 1> c{};
    math::Vector<float, 1> u{};
    math::Vector<float, 1> y{};

    for (int k{ 0 }; k < kSteps; ++k)
    {
        y = nominal.Output(x, u);
        u = dob.Compute(c, y);
        math::Vector<float, 1> uActual{};
        uActual.at(0, 0) = u.at(0, 0) + kDisturbance;
        x = nominal.Step(x, uActual);
    }

    EXPECT_GT(std::abs(dob.Disturbance().at(0, 0)), 0.1f);

    dob.Reset();

    EXPECT_NEAR(dob.Disturbance().at(0, 0), 0.0f, math::Tolerance<float>());
}

TEST_F(TestDisturbanceObserver, robust_to_small_model_mismatch)
{
    static constexpr float kMismatchFactor{ 0.95f };
    static constexpr float kDisturbance{ 0.5f };
    static constexpr int kSteps{ 4000 };

    math::LinearTimeInvariant<float, 1, 1, 1> truePlant{ MakeFirstOrderPlant() };
    truePlant.A.at(0, 0) *= kMismatchFactor;

    math::Vector<float, 1> x{};
    math::Vector<float, 1> c{};
    math::Vector<float, 1> u{};
    math::Vector<float, 1> y{};

    for (int k{ 0 }; k < kSteps; ++k)
    {
        y = truePlant.Output(x, u);
        u = dob.Compute(c, y);
        math::Vector<float, 1> uActual{};
        uActual.at(0, 0) = u.at(0, 0) + kDisturbance;
        x = truePlant.Step(x, uActual);
    }

    EXPECT_LT(std::abs(dob.Disturbance().at(0, 0) - kDisturbance), kDisturbance);
}
