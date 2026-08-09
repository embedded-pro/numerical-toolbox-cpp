#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/controllers/implementations/IntegralStateFeedbackLqi.hpp"
#include "numerical/math/LinearTimeInvariant.hpp"
#include "numerical/math/Tolerance.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    using Plant = math::LinearTimeInvariant<float, 2, 1, 1>;
    using Controller = controllers::IntegralStateFeedbackLqi<float, 2, 1, 1>;

    static Plant MakeFirstOrderPlant()
    {
        Plant p{};
        p.A = math::SquareMatrix<float, 2>{
            { 1.0f, 0.01f },
            { 0.0f, 1.0f }
        };
        p.B = math::Matrix<float, 2, 1>{
            { 0.0f },
            { 0.01f }
        };
        p.C = math::Matrix<float, 1, 2>{ { 1.0f, 0.0f } };
        return p;
    }

    class TestIntegralStateFeedbackLqi : public ::testing::Test
    {
    protected:
        Plant plant{ MakeFirstOrderPlant() };
        math::SquareMatrix<float, 3> Q{
            { 100.0f, 0.0f, 0.0f },
            { 0.0f, 1.0f, 0.0f },
            { 0.0f, 0.0f, 1000.0f }
        };
        math::SquareMatrix<float, 1> R{ { 0.01f } };
        Controller controller{ plant, Q, R, 0.01f };
    };
}

TEST_F(TestIntegralStateFeedbackLqi, zero_steady_state_error_to_step_reference)
{
    math::Vector<float, 2> x{};
    math::Vector<float, 1> reference{ { 1.0f } };

    for (int k = 0; k < 500; ++k)
    {
        auto y = plant.Output(x, math::Vector<float, 1>{});
        auto u = controller.ComputeControl(x, reference, y);
        x = plant.Step(x, u);
    }

    auto y = plant.Output(x, math::Vector<float, 1>{});
    EXPECT_NEAR(y.at(0, 0), 1.0f, math::Tolerance<float>());
}

TEST_F(TestIntegralStateFeedbackLqi, rejects_constant_output_disturbance)
{
    math::Vector<float, 2> x{};
    math::Vector<float, 1> reference{ { 1.0f } };
    math::Vector<float, 1> disturbance{ { 0.2f } };

    for (int k = 0; k < 600; ++k)
    {
        auto y = plant.Output(x, math::Vector<float, 1>{}) + disturbance;
        auto u = controller.ComputeControl(x, reference, y);
        x = plant.Step(x, u);
    }

    auto y = plant.Output(x, math::Vector<float, 1>{}) + disturbance;
    EXPECT_NEAR(y.at(0, 0), 1.0f, math::Tolerance<float>());
}

TEST_F(TestIntegralStateFeedbackLqi, integral_accumulates_error)
{
    math::Vector<float, 2> x{};
    math::Vector<float, 1> reference{ { 0.5f } };
    math::Vector<float, 1> measured{ { 0.0f } };

    float expected = 0.0f;
    constexpr int steps = 10;
    for (int k = 0; k < steps; ++k)
    {
        expected += (reference.at(0, 0) - measured.at(0, 0)) * 0.01f;
        controller.ComputeControl(x, reference, measured);
    }

    math::Vector<float, 1> zeroRef{ { 0.0f } };
    math::Vector<float, 1> zeroMeas{ { 0.0f } };
    auto u = controller.ComputeControl(x, zeroRef, zeroMeas);
    float ki0 = controller.GetGainIntegral().at(0, 0);
    EXPECT_NEAR(u.at(0, 0), -ki0 * expected, math::Tolerance<float>());
}

TEST_F(TestIntegralStateFeedbackLqi, gain_split_matches_augmented_lqr)
{
    using AugLqr = controllers::Lqr<float, 3, 1>;

    math::SquareMatrix<float, 3> Aa{};
    math::Matrix<float, 3, 1> Ba{};

    Aa.at(0, 0) = plant.A.at(0, 0);
    Aa.at(0, 1) = plant.A.at(0, 1);
    Aa.at(1, 0) = plant.A.at(1, 0);
    Aa.at(1, 1) = plant.A.at(1, 1);
    Aa.at(2, 0) = -plant.C.at(0, 0) * 0.01f;
    Aa.at(2, 1) = -plant.C.at(0, 1) * 0.01f;
    Aa.at(2, 2) = 1.0f;

    Ba.at(0, 0) = plant.B.at(0, 0);
    Ba.at(1, 0) = plant.B.at(1, 0);

    AugLqr lqr{ Aa, Ba, Q, R };
    const auto& Ka = lqr.GetGain();

    EXPECT_NEAR(controller.GetGainState().at(0, 0), Ka.at(0, 0), math::Tolerance<float>());
    EXPECT_NEAR(controller.GetGainState().at(0, 1), Ka.at(0, 1), math::Tolerance<float>());
    EXPECT_NEAR(controller.GetGainIntegral().at(0, 0), Ka.at(0, 2), math::Tolerance<float>());
}

TEST_F(TestIntegralStateFeedbackLqi, closed_loop_is_stable)
{
    math::Vector<float, 2> x{ { 2.0f }, { 0.5f } };
    math::Vector<float, 1> reference{ { 0.0f } };

    for (int k = 0; k < 500; ++k)
    {
        auto y = plant.Output(x, math::Vector<float, 1>{});
        auto u = controller.ComputeControl(x, reference, y);
        x = plant.Step(x, u);
    }

    EXPECT_LT(std::abs(x.at(0, 0)), 1.0f);
    EXPECT_LT(std::abs(x.at(1, 0)), 1.0f);
}

TEST_F(TestIntegralStateFeedbackLqi, reset_clears_integral)
{
    math::Vector<float, 2> x{};
    math::Vector<float, 1> reference{ { 1.0f } };
    math::Vector<float, 1> measured{ { 0.0f } };

    for (int k = 0; k < 20; ++k)
        controller.ComputeControl(x, reference, measured);

    controller.Reset();

    auto u = controller.ComputeControl(x, measured, measured);
    float kx0 = controller.GetGainState().at(0, 0);
    float kx1 = controller.GetGainState().at(0, 1);
    EXPECT_NEAR(u.at(0, 0), -(kx0 * x.at(0, 0) + kx1 * x.at(1, 0)), math::Tolerance<float>());
}

TEST_F(TestIntegralStateFeedbackLqi, reference_change_tracked)
{
    math::Vector<float, 2> x{};
    math::Vector<float, 1> ref1{ { 1.0f } };
    math::Vector<float, 1> ref2{ { 2.0f } };

    for (int k = 0; k < 400; ++k)
    {
        auto y = plant.Output(x, math::Vector<float, 1>{});
        auto u = controller.ComputeControl(x, ref1, y);
        x = plant.Step(x, u);
    }

    for (int k = 0; k < 500; ++k)
    {
        auto y = plant.Output(x, math::Vector<float, 1>{});
        auto u = controller.ComputeControl(x, ref2, y);
        x = plant.Step(x, u);
    }

    auto y = plant.Output(x, math::Vector<float, 1>{});
    EXPECT_NEAR(y.at(0, 0), 2.0f, math::Tolerance<float>());
}

TEST_F(TestIntegralStateFeedbackLqi, control_uses_negative_feedback)
{
    math::Vector<float, 2> x{ { 1.0f }, { 0.0f } };
    math::Vector<float, 1> reference{ { 0.0f } };
    math::Vector<float, 1> measured{ { 0.0f } };

    controller.Reset();

    auto u = controller.ComputeControl(x, reference, measured);

    float kx0 = controller.GetGainState().at(0, 0);
    float kx1 = controller.GetGainState().at(0, 1);
    float expected = -(kx0 * x.at(0, 0) + kx1 * x.at(1, 0));
    EXPECT_NEAR(u.at(0, 0), expected, math::Tolerance<float>());
    EXPECT_LT(u.at(0, 0), 0.0f);
}

TEST_F(TestIntegralStateFeedbackLqi, direct_gain_constructor_matches_lqr_constructor)
{
    const auto kx = controller.GetGainState();
    const auto ki = controller.GetGainIntegral();

    Controller directController{ kx, ki, 0.01f };

    math::Vector<float, 2> x{ { 0.5f }, { 0.1f } };
    math::Vector<float, 1> reference{ { 1.0f } };
    math::Vector<float, 1> measured{ { 0.3f } };

    auto u1 = controller.ComputeControl(x, reference, measured);
    auto u2 = directController.ComputeControl(x, reference, measured);

    EXPECT_NEAR(u1.at(0, 0), u2.at(0, 0), math::Tolerance<float>());
}

TEST_F(TestIntegralStateFeedbackLqi, direct_gain_constructor_get_gains)
{
    math::Matrix<float, 1, 2> kx{ { 1.5f, 0.8f } };
    math::Matrix<float, 1, 1> ki{ { 2.0f } };

    Controller c{ kx, ki, 0.01f };

    EXPECT_FLOAT_EQ(c.GetGainState().at(0, 0), 1.5f);
    EXPECT_FLOAT_EQ(c.GetGainState().at(0, 1), 0.8f);
    EXPECT_FLOAT_EQ(c.GetGainIntegral().at(0, 0), 2.0f);
}
