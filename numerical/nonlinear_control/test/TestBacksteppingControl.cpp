#include "numerical/math/Tolerance.hpp"
#include "numerical/nonlinear_control/BacksteppingControl.hpp"
#include <array>
#include <cmath>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace
{
    template<typename T, std::size_t Order>
    class MockStrictFeedbackModel
        : public nonlinear_control::StrictFeedbackModel<T, Order>
    {
    public:
        using StateVector = std::array<T, Order>;
        MOCK_METHOD(T, Drift, (std::size_t, const StateVector&), (const, override));
        MOCK_METHOD(T, Gain, (std::size_t, const StateVector&), (const, override));
        MOCK_METHOD(T, VirtualDerivative, (std::size_t, const StateVector&, T), (const, override));
    };

    class TestBacksteppingControlOrder1 : public ::testing::Test
    {
    protected:
        ::testing::StrictMock<MockStrictFeedbackModel<float, 1>> model;
        std::array<float, 1> gains{ { 2.0f } };
        nonlinear_control::BacksteppingControl<float, 1> controller{ model, gains };
    };

    class TestBacksteppingControlOrder2 : public ::testing::Test
    {
    protected:
        ::testing::StrictMock<MockStrictFeedbackModel<float, 2>> model;
        std::array<float, 2> gains{ { 2.0f, 3.0f } };
        nonlinear_control::BacksteppingControl<float, 2> controller{ model, gains };
    };

    template<typename T>
    class PendulumPlant
        : public nonlinear_control::StrictFeedbackModel<T, 1>
    {
    public:
        using StateVector = std::array<T, 1>;

        T Drift(std::size_t, const StateVector& x) const override
        {
            return std::sin(x[0]);
        }

        T Gain(std::size_t, const StateVector&) const override
        {
            return T{ 1 };
        }

        T VirtualDerivative(std::size_t, const StateVector&, T) const override
        {
            return T{ 0 };
        }
    };

    template<typename T>
    class DoubleIntegratorPlant
        : public nonlinear_control::StrictFeedbackModel<T, 2>
    {
    public:
        using StateVector = std::array<T, 2>;

        explicit DoubleIntegratorPlant(const std::array<T, 2>& gains)
            : gains{ gains }
        {}

        T Drift(std::size_t, const StateVector&) const override
        {
            return T{ 0 };
        }

        T Gain(std::size_t, const StateVector&) const override
        {
            return T{ 1 };
        }

        T VirtualDerivative(std::size_t, const StateVector& x, T) const override
        {
            return -gains[0] * x[1];
        }

    private:
        std::array<T, 2> gains;
    };

    class TestBacksteppingRealPlantOrder1 : public ::testing::Test
    {
    protected:
        PendulumPlant<float> plant;
        std::array<float, 1> gains{ { 3.0f } };
        nonlinear_control::BacksteppingControl<float, 1> controller{ plant, gains };
    };

    class TestBacksteppingRealPlantOrder2 : public ::testing::Test
    {
    protected:
        std::array<float, 2> gains{ { 3.0f, 4.0f } };
        DoubleIntegratorPlant<float> plant{ gains };
        nonlinear_control::BacksteppingControl<float, 2> controller{ plant, gains };
    };
}

TEST_F(TestBacksteppingControlOrder1, single_stage_reduces_to_proportional)
{
    const float e{ 0.5f };
    std::array<float, 1> x{ { e } };
    nonlinear_control::BacksteppingControl<float, 1>::Reference ref{ 0.0f, 0.0f };

    EXPECT_CALL(model, Drift(0, x)).WillOnce(::testing::Return(0.0f));
    EXPECT_CALL(model, Gain(0, x)).WillOnce(::testing::Return(1.0f));

    const float u = controller.ComputeControl(x, ref);

    EXPECT_NEAR(u, -gains[0] * e, math::Tolerance<float>());
}

TEST_F(TestBacksteppingControlOrder1, cancels_stage_drift)
{
    const float d{ 1.5f };
    std::array<float, 1> x{ { 0.0f } };
    nonlinear_control::BacksteppingControl<float, 1>::Reference ref{ 0.0f, 0.0f };

    EXPECT_CALL(model, Drift(0, x)).WillOnce(::testing::Return(d));
    EXPECT_CALL(model, Gain(0, x)).WillOnce(::testing::Return(1.0f));

    const float u = controller.ComputeControl(x, ref);

    EXPECT_NEAR(u, -d, math::Tolerance<float>());
}

TEST_F(TestBacksteppingControlOrder1, divides_by_control_gain)
{
    const float e{ 0.4f };
    const float g{ 2.0f };
    std::array<float, 1> x{ { e } };
    nonlinear_control::BacksteppingControl<float, 1>::Reference ref{ 0.0f, 0.0f };

    EXPECT_CALL(model, Drift(0, x)).WillOnce(::testing::Return(0.0f));
    EXPECT_CALL(model, Gain(0, x)).WillOnce(::testing::Return(g));

    const float u = controller.ComputeControl(x, ref);

    EXPECT_NEAR(u, (-gains[0] * e) / g, math::Tolerance<float>());
}

TEST_F(TestBacksteppingControlOrder2, two_stage_includes_cross_term)
{
    const float z1{ 0.3f };
    const float z2{ 0.2f };
    const float g0{ 1.5f };
    const float g1{ 2.0f };
    const float f1{ 0.5f };
    const float alphaDot1{ 0.1f };

    std::array<float, 2> x{ { z1, z1 + z2 } };
    nonlinear_control::BacksteppingControl<float, 2>::Reference ref{ 0.0f, 0.0f };

    ::testing::InSequence seq;
    EXPECT_CALL(model, Drift(0, x)).WillOnce(::testing::Return(0.0f));
    EXPECT_CALL(model, Gain(0, x)).WillOnce(::testing::Return(g0));
    EXPECT_CALL(model, VirtualDerivative(0, x, ::testing::_)).WillOnce(::testing::Return(alphaDot1));
    EXPECT_CALL(model, Drift(1, x)).WillOnce(::testing::Return(f1));
    EXPECT_CALL(model, Gain(1, x)).WillOnce(::testing::Return(g1));
    EXPECT_CALL(model, Gain(0, x)).WillOnce(::testing::Return(g0));

    const float u = controller.ComputeControl(x, ref);

    const float alpha0 = (-gains[0] * z1) / g0;
    const float z2Actual = x[1] - alpha0;
    const float expectedU = (alphaDot1 - f1 - gains[1] * z2Actual - g0 * z1) / g1;

    EXPECT_NEAR(u, expectedU, math::Tolerance<float>());
}

TEST_F(TestBacksteppingControlOrder2, virtual_control_feeds_next_stage)
{
    const float x1{ 1.0f };
    const float x2{ 0.5f };
    const float g0{ 1.0f };

    std::array<float, 2> x{ { x1, x2 } };
    nonlinear_control::BacksteppingControl<float, 2>::Reference ref{ 0.0f, 0.0f };

    ::testing::InSequence seq;
    EXPECT_CALL(model, Drift(0, x)).WillOnce(::testing::Return(0.0f));
    EXPECT_CALL(model, Gain(0, x)).WillOnce(::testing::Return(g0));
    EXPECT_CALL(model, VirtualDerivative(0, x, ::testing::_)).WillOnce(::testing::Return(0.0f));
    EXPECT_CALL(model, Drift(1, x)).WillOnce(::testing::Return(0.0f));
    EXPECT_CALL(model, Gain(1, x)).WillOnce(::testing::Return(1.0f));
    EXPECT_CALL(model, Gain(0, x)).WillOnce(::testing::Return(g0));

    controller.ComputeControl(x, ref);

    const float alpha0 = (-gains[0] * x1) / g0;
    const float z2Expected = x2 - alpha0;

    EXPECT_NEAR(x2 - alpha0, z2Expected, math::Tolerance<float>());
}

TEST_F(TestBacksteppingControlOrder1, reference_derivative_feedforward)
{
    const float rdot{ 0.8f };
    std::array<float, 1> x{ { 0.0f } };
    nonlinear_control::BacksteppingControl<float, 1>::Reference ref{ 0.0f, rdot };

    EXPECT_CALL(model, Drift(0, x)).WillOnce(::testing::Return(0.0f));
    EXPECT_CALL(model, Gain(0, x)).WillOnce(::testing::Return(1.0f));

    const float u = controller.ComputeControl(x, ref);

    EXPECT_NEAR(u, rdot, math::Tolerance<float>());
}

TEST_F(TestBacksteppingControlOrder2, lyapunov_decreases_closed_loop)
{
    std::array<float, 2> x{ { 1.0f, 0.0f } };
    nonlinear_control::BacksteppingControl<float, 2>::Reference ref{ 0.0f, 0.0f };

    const float dt{ 0.01f };
    const float alpha0Init = -gains[0] * x[0];
    const float z1Init = x[0];
    const float z2Init = x[1] - alpha0Init;
    float vPrev{ 0.5f * (z1Init * z1Init + z2Init * z2Init) };

    for (int step = 0; step < 50; ++step)
    {
        const float alphaDotExpected = -gains[0] * x[1];
        {
            ::testing::InSequence seq;
            EXPECT_CALL(model, Drift(0, x)).WillOnce(::testing::Return(0.0f));
            EXPECT_CALL(model, Gain(0, x)).WillOnce(::testing::Return(1.0f));
            EXPECT_CALL(model, VirtualDerivative(0, x, ::testing::_)).WillOnce(::testing::Return(alphaDotExpected));
            EXPECT_CALL(model, Drift(1, x)).WillOnce(::testing::Return(0.0f));
            EXPECT_CALL(model, Gain(1, x)).WillOnce(::testing::Return(1.0f));
            EXPECT_CALL(model, Gain(0, x)).WillOnce(::testing::Return(1.0f));
        }

        const float u = controller.ComputeControl(x, ref);

        x[0] += dt * x[1];
        x[1] += dt * u;

        const float alpha0New = -gains[0] * x[0];
        const float z1New = x[0];
        const float z2New = x[1] - alpha0New;
        const float vNew = 0.5f * (z1New * z1New + z2New * z2New);

        EXPECT_LT(vNew, vPrev + math::Tolerance<float>());
        vPrev = vNew;
        (void)u;
    }
}

TEST_F(TestBacksteppingControlOrder1, gains_set_convergence_rate)
{
    std::array<float, 1> gainsLow{ { 1.0f } };
    std::array<float, 1> gainsHigh{ { 5.0f } };

    ::testing::StrictMock<MockStrictFeedbackModel<float, 1>> modelLow;
    ::testing::StrictMock<MockStrictFeedbackModel<float, 1>> modelHigh;

    nonlinear_control::BacksteppingControl<float, 1> ctrlLow{ modelLow, gainsLow };
    nonlinear_control::BacksteppingControl<float, 1> ctrlHigh{ modelHigh, gainsHigh };

    const float dt{ 0.01f };
    const float tol{ 0.05f };
    float xLow{ 1.0f };
    float xHigh{ 1.0f };
    int stepsLow{ 0 };
    int stepsHigh{ 0 };

    for (int i = 0; i < 500 && xLow > tol; ++i)
    {
        std::array<float, 1> state{ { xLow } };
        nonlinear_control::BacksteppingControl<float, 1>::Reference ref{ 0.0f, 0.0f };
        EXPECT_CALL(modelLow, Drift(0, state)).WillOnce(::testing::Return(0.0f));
        EXPECT_CALL(modelLow, Gain(0, state)).WillOnce(::testing::Return(1.0f));
        const float u = ctrlLow.ComputeControl(state, ref);
        xLow += dt * u;
        ++stepsLow;
    }

    for (int i = 0; i < 500 && xHigh > tol; ++i)
    {
        std::array<float, 1> state{ { xHigh } };
        nonlinear_control::BacksteppingControl<float, 1>::Reference ref{ 0.0f, 0.0f };
        EXPECT_CALL(modelHigh, Drift(0, state)).WillOnce(::testing::Return(0.0f));
        EXPECT_CALL(modelHigh, Gain(0, state)).WillOnce(::testing::Return(1.0f));
        const float u = ctrlHigh.ComputeControl(state, ref);
        xHigh += dt * u;
        ++stepsHigh;
    }

    EXPECT_LT(stepsHigh, stepsLow);
}

TEST_F(TestBacksteppingRealPlantOrder1, cancels_nonlinearity_and_regulates)
{
    float x{ 1.0f };
    const float dt{ 0.001f };
    nonlinear_control::BacksteppingControl<float, 1>::Reference ref{ 0.0f, 0.0f };

    for (int i = 0; i < 5000; ++i)
    {
        std::array<float, 1> state{ { x } };
        const float u = controller.ComputeControl(state, ref);
        x += dt * (std::sin(x) + u);
    }

    EXPECT_NEAR(x, 0.0f, 1.0e-2f);
}

TEST_F(TestBacksteppingRealPlantOrder1, tracks_constant_reference)
{
    float x{ 0.0f };
    const float dt{ 0.001f };
    nonlinear_control::BacksteppingControl<float, 1>::Reference ref{ 0.7f, 0.0f };

    for (int i = 0; i < 5000; ++i)
    {
        std::array<float, 1> state{ { x } };
        const float u = controller.ComputeControl(state, ref);
        x += dt * (std::sin(x) + u);
    }

    EXPECT_NEAR(x, ref.value, 1.0e-2f);
}

TEST_F(TestBacksteppingRealPlantOrder2, reset_restores_initial_response)
{
    std::array<float, 2> x{ { 0.4f, -0.2f } };
    nonlinear_control::BacksteppingControl<float, 2>::Reference ref{ 0.0f, 0.0f };

    const float first = controller.ComputeControl(x, ref);

    std::array<float, 2> drift{ { 1.0f, 0.6f } };
    controller.ComputeControl(drift, ref);
    controller.Reset();

    const float afterReset = controller.ComputeControl(x, ref);

    EXPECT_NEAR(afterReset, first, math::Tolerance<float>());
}

TEST_F(TestBacksteppingRealPlantOrder2, stabilizes_double_integrator)
{
    std::array<float, 2> x{ { 1.0f, -0.5f } };
    const float dt{ 0.001f };
    nonlinear_control::BacksteppingControl<float, 2>::Reference ref{ 0.0f, 0.0f };

    for (int i = 0; i < 20000; ++i)
    {
        const float u = controller.ComputeControl(x, ref);
        x[0] += dt * x[1];
        x[1] += dt * u;
    }

    EXPECT_NEAR(x[0], 0.0f, 1.0e-2f);
    EXPECT_NEAR(x[1], 0.0f, 1.0e-2f);
}

TEST_F(TestBacksteppingRealPlantOrder1, reset_restores_initial_response_order1)
{
    std::array<float, 1> x{ { 0.8f } };
    nonlinear_control::BacksteppingControl<float, 1>::Reference ref{ 0.0f, 0.0f };

    const float first = controller.ComputeControl(x, ref);

    std::array<float, 1> xOther{ { 2.0f } };
    controller.ComputeControl(xOther, ref);
    controller.Reset();

    const float afterReset = controller.ComputeControl(x, ref);

    EXPECT_NEAR(afterReset, first, math::Tolerance<float>());
}

TEST_F(TestBacksteppingControlOrder1, zero_error_output_equals_negative_drift_over_gain)
{
    const float d{ 1.2f };
    const float g{ 2.0f };
    std::array<float, 1> x{ { 0.5f } };
    nonlinear_control::BacksteppingControl<float, 1>::Reference ref{ 0.5f, 0.0f };

    EXPECT_CALL(model, Drift(0, x)).WillOnce(::testing::Return(d));
    EXPECT_CALL(model, Gain(0, x)).WillOnce(::testing::Return(g));

    const float u = controller.ComputeControl(x, ref);

    EXPECT_NEAR(u, -d / g, math::Tolerance<float>());
}

TEST_F(TestBacksteppingControlOrder1, drift_and_error_combined_output)
{
    const float e{ 0.6f };
    const float d{ 1.0f };
    const float g{ 2.5f };
    std::array<float, 1> x{ { e } };
    nonlinear_control::BacksteppingControl<float, 1>::Reference ref{ 0.0f, 0.0f };

    EXPECT_CALL(model, Drift(0, x)).WillOnce(::testing::Return(d));
    EXPECT_CALL(model, Gain(0, x)).WillOnce(::testing::Return(g));

    const float u = controller.ComputeControl(x, ref);

    EXPECT_NEAR(u, (-d - gains[0] * e) / g, math::Tolerance<float>());
}

TEST_F(TestBacksteppingControlOrder1, two_instances_do_not_share_state)
{
    ::testing::StrictMock<MockStrictFeedbackModel<float, 1>> modelB;
    std::array<float, 1> gainsB{ { 2.0f } };
    nonlinear_control::BacksteppingControl<float, 1> ctrlB{ modelB, gainsB };

    std::array<float, 1> xA{ { 1.0f } };
    std::array<float, 1> xB{ { 1.0f } };
    nonlinear_control::BacksteppingControl<float, 1>::Reference ref{ 0.0f, 0.0f };

    EXPECT_CALL(model, Drift(0, xA)).WillOnce(::testing::Return(0.0f));
    EXPECT_CALL(model, Gain(0, xA)).WillOnce(::testing::Return(1.0f));
    EXPECT_CALL(modelB, Drift(0, xB)).WillOnce(::testing::Return(0.0f));
    EXPECT_CALL(modelB, Gain(0, xB)).WillOnce(::testing::Return(1.0f));

    const float uA = controller.ComputeControl(xA, ref);
    const float uB = ctrlB.ComputeControl(xB, ref);

    EXPECT_FLOAT_EQ(uA, uB);
}

TEST_F(TestBacksteppingControlOrder1, large_state_produces_finite_output)
{
    const float large{ 1.0e6f };
    std::array<float, 1> x{ { large } };
    nonlinear_control::BacksteppingControl<float, 1>::Reference ref{ 0.0f, 0.0f };

    EXPECT_CALL(model, Drift(0, x)).WillOnce(::testing::Return(0.0f));
    EXPECT_CALL(model, Gain(0, x)).WillOnce(::testing::Return(1.0f));

    const float u = controller.ComputeControl(x, ref);

    EXPECT_TRUE(std::isfinite(u));
    EXPECT_NEAR(u, -gains[0] * large, 1.0f);
}

TEST_F(TestBacksteppingControlOrder1, determinism_same_input_same_output)
{
    std::array<float, 1> x{ { 0.7f } };
    nonlinear_control::BacksteppingControl<float, 1>::Reference ref{ 0.2f, 0.1f };

    EXPECT_CALL(model, Drift(0, x)).Times(2).WillRepeatedly(::testing::Return(0.3f));
    EXPECT_CALL(model, Gain(0, x)).Times(2).WillRepeatedly(::testing::Return(1.5f));

    controller.Reset();
    const float u1 = controller.ComputeControl(x, ref);
    controller.Reset();
    const float u2 = controller.ComputeControl(x, ref);

    EXPECT_FLOAT_EQ(u1, u2);
}
