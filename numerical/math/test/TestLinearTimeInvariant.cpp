#include "numerical/math/LinearTimeInvariant.hpp"
#include "numerical/math/Tolerance.hpp"
#include <gtest/gtest.h>

namespace
{
    class LinearTimeInvariantTest : public ::testing::Test
    {
    };
}

TEST_F(LinearTimeInvariantTest, DefaultConstructionZerosAllMatrices)
{
    math::LinearTimeInvariant<float, 2, 1> lti;

    EXPECT_NEAR(lti.A.at(0, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(lti.A.at(0, 1), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(lti.A.at(1, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(lti.A.at(1, 1), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(lti.B.at(0, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(lti.B.at(1, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(lti.C.at(0, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(lti.C.at(0, 1), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(lti.D.at(0, 0), 0.0f, math::Tolerance<float>());
}

TEST_F(LinearTimeInvariantTest, StepReturnsAxPlusBu)
{
    math::LinearTimeInvariant<float, 2, 1> lti;
    lti.A = math::SquareMatrix<float, 2>{
        { 0.9f, 0.5f },
        { 0.0f, 0.9f }
    };
    lti.B = math::Matrix<float, 2, 1>{
        { 0.0f },
        { 0.1f }
    };

    math::Vector<float, 2> x{ { 0.5f }, { 0.25f } };
    math::Vector<float, 1> u{ { 0.1f } };

    auto xNext = lti.Step(x, u);

    EXPECT_NEAR(xNext.at(0, 0), 0.9f * 0.5f + 0.5f * 0.25f, math::Tolerance<float>());
    EXPECT_NEAR(xNext.at(1, 0), 0.9f * 0.25f + 0.1f * 0.1f, math::Tolerance<float>());
}

TEST_F(LinearTimeInvariantTest, OutputReturnsCxPlusDu)
{
    math::LinearTimeInvariant<float, 2, 1, 1> lti;
    lti.C = math::Matrix<float, 1, 2>{ { 0.9f, 0.0f } };
    lti.D = math::Matrix<float, 1, 1>{ { 0.0f } };

    math::Vector<float, 2> x{ { 0.5f }, { 0.25f } };
    math::Vector<float, 1> u{ { 0.1f } };

    auto y = lti.Output(x, u);

    EXPECT_NEAR(y.at(0, 0), 0.9f * 0.5f, math::Tolerance<float>());
}

TEST_F(LinearTimeInvariantTest, WithFullStateOutputSetsCIdentityAndDZero)
{
    auto A = math::SquareMatrix<float, 2>{
        { 0.9f, 0.1f },
        { 0.0f, 0.8f }
    };
    auto B = math::Matrix<float, 2, 1>{
        { 0.0f },
        { 0.1f }
    };

    auto lti = math::LinearTimeInvariant<float, 2, 1>::WithFullStateOutput(A, B);

    EXPECT_NEAR(lti.C.at(0, 0), 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(lti.C.at(1, 1), 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(lti.C.at(0, 1), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(lti.C.at(1, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(lti.D.at(0, 0), 0.0f, math::Tolerance<float>());
}

TEST_F(LinearTimeInvariantTest, AutonomousFactoryZerosB)
{
    auto A = math::SquareMatrix<float, 2>{
        { 0.9f, 0.0f },
        { 0.0f, 0.8f }
    };
    auto C = math::Matrix<float, 1, 2>{ { 0.9f, 0.0f } };

    auto lti = math::LinearTimeInvariant<float, 2, 1, 1>::Autonomous(A, C);

    EXPECT_NEAR(lti.B.at(0, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(lti.B.at(1, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(lti.D.at(0, 0), 0.0f, math::Tolerance<float>());
}

TEST_F(LinearTimeInvariantTest, MultiStepPropagatesStateCorrectly)
{
    auto lti = math::LinearTimeInvariant<float, 2, 1>::WithFullStateOutput(
        math::SquareMatrix<float, 2>{
            { 1.0f, 0.1f },
            { 0.0f, 0.9f } },
        math::Matrix<float, 2, 1>{
            { 0.0f },
            { 0.1f } });

    math::Vector<float, 2> x{ { 0.0f }, { 0.0f } };
    math::Vector<float, 1> u{ { 0.5f } };

    x = lti.Step(x, u);
    x = lti.Step(x, u);

    EXPECT_NEAR(x.at(0, 0), 0.0f * 1.0f + 0.05f * 0.1f, math::Tolerance<float>());
    EXPECT_NEAR(x.at(1, 0), 0.9f * 0.05f + 0.1f * 0.5f, math::Tolerance<float>());
}

TEST_F(LinearTimeInvariantTest, ThreeStateStepAndOutput)
{
    auto lti = math::LinearTimeInvariant<float, 3, 1>::WithFullStateOutput(
        math::SquareMatrix<float, 3>{
            { 0.9f, 0.1f, 0.0f },
            { 0.0f, 0.8f, 0.1f },
            { 0.0f, 0.0f, 0.7f } },
        math::Matrix<float, 3, 1>{ { 0.0f }, { 0.0f }, { 0.1f } });

    math::Vector<float, 3> x{ { 1.0f }, { 0.5f }, { 0.25f } };
    math::Vector<float, 1> u{ { 2.0f } };

    auto xNext = lti.Step(x, u);
    EXPECT_NEAR(xNext.at(0, 0), 0.9f * 1.0f + 0.1f * 0.5f, math::Tolerance<float>());
    EXPECT_NEAR(xNext.at(1, 0), 0.8f * 0.5f + 0.1f * 0.25f, math::Tolerance<float>());
    EXPECT_NEAR(xNext.at(2, 0), 0.7f * 0.25f + 0.1f * 2.0f, math::Tolerance<float>());

    auto y = lti.Output(x, u);
    EXPECT_NEAR(y.at(0, 0), x.at(0, 0), math::Tolerance<float>());
    EXPECT_NEAR(y.at(1, 0), x.at(1, 0), math::Tolerance<float>());
    EXPECT_NEAR(y.at(2, 0), x.at(2, 0), math::Tolerance<float>());
}

TEST_F(LinearTimeInvariantTest, FourStateAutonomousStep)
{
    auto A = math::SquareMatrix<float, 4>{
        { 0.9f, 0.0f, 0.0f, 0.0f },
        { 0.0f, 0.8f, 0.0f, 0.0f },
        { 0.0f, 0.0f, 0.7f, 0.0f },
        { 0.0f, 0.0f, 0.0f, 0.6f }
    };
    auto C = math::Matrix<float, 1, 4>{ { 1.0f, 0.0f, 0.0f, 0.0f } };

    auto lti = math::LinearTimeInvariant<float, 4, 1, 1>::Autonomous(A, C);

    math::Vector<float, 4> x{ { 1.0f }, { 0.5f }, { 0.25f }, { 0.1f } };
    math::Vector<float, 1> u{};

    auto xNext = lti.Step(x, u);
    EXPECT_NEAR(xNext.at(0, 0), 0.9f, math::Tolerance<float>());
    EXPECT_NEAR(xNext.at(1, 0), 0.8f * 0.5f, math::Tolerance<float>());
    EXPECT_NEAR(xNext.at(2, 0), 0.7f * 0.25f, math::Tolerance<float>());
    EXPECT_NEAR(xNext.at(3, 0), 0.6f * 0.1f, math::Tolerance<float>());
}

TEST_F(LinearTimeInvariantTest, TwoStateMultiInputStep)
{
    auto lti = math::LinearTimeInvariant<float, 2, 2>::WithFullStateOutput(
        math::SquareMatrix<float, 2>{
            { 0.9f, 0.0f },
            { 0.0f, 0.8f } },
        math::Matrix<float, 2, 2>{
            { 0.1f, 0.0f },
            { 0.0f, 0.1f } });

    math::Vector<float, 2> x{ { 1.0f }, { 0.5f } };
    math::Vector<float, 2> u{ { 1.0f }, { 2.0f } };

    auto xNext = lti.Step(x, u);
    EXPECT_NEAR(xNext.at(0, 0), 0.9f * 1.0f + 0.1f * 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(xNext.at(1, 0), 0.8f * 0.5f + 0.1f * 2.0f, math::Tolerance<float>());
}

namespace
{
    class TestLinearTimeInvariantShapes : public ::testing::Test
    {
    protected:
        template<std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>
        static math::LinearTimeInvariant<float, StateSize, InputSize, OutputSize> MakeChain()
        {
            math::LinearTimeInvariant<float, StateSize, InputSize, OutputSize> lti{};

            for (std::size_t i = 0; i < StateSize; ++i)
                lti.A.at(i, i) = 0.5f;

            for (std::size_t i = 0; i + 1 < StateSize; ++i)
                lti.A.at(i, i + 1) = 1.0f;

            for (std::size_t i = 0; i < InputSize; ++i)
                lti.B.at(i, i) = 1.0f;

            for (std::size_t i = 0; i < OutputSize && i < StateSize; ++i)
                lti.C.at(i, i) = 1.0f;

            return lti;
        }
    };
}

TEST_F(TestLinearTimeInvariantShapes, step_and_output_agree_with_matrix_products)
{
    auto lti = MakeChain<3, 1, 3>();

    math::Vector<float, 3> x{};
    x.at(0, 0) = 1.0f;
    x.at(1, 0) = 2.0f;
    x.at(2, 0) = -1.0f;

    math::Vector<float, 1> u{};
    u.at(0, 0) = 0.5f;

    const auto next = lti.Step(x, u);
    const auto expected = lti.A * x + lti.B * u;

    for (std::size_t i = 0; i < 3; ++i)
        EXPECT_NEAR(next.at(i, 0), expected.at(i, 0), math::Tolerance<float>());

    const auto y = lti.Output(x, u);
    const auto expectedOutput = lti.C * x + lti.D * u;

    for (std::size_t i = 0; i < 3; ++i)
        EXPECT_NEAR(y.at(i, 0), expectedOutput.at(i, 0), math::Tolerance<float>());
}

TEST_F(TestLinearTimeInvariantShapes, single_output_shape_reduces_state_to_scalar)
{
    auto lti = MakeChain<2, 1, 1>();

    math::Vector<float, 2> x{};
    x.at(0, 0) = 3.0f;
    x.at(1, 0) = 4.0f;

    math::Vector<float, 1> u{};
    u.at(0, 0) = 1.0f;

    const auto y = lti.Output(x, u);
    EXPECT_NEAR(y.at(0, 0), 3.0f, math::Tolerance<float>());

    const auto next = lti.Step(x, u);
    EXPECT_NEAR(next.at(0, 0), 0.5f * 3.0f + 4.0f + 1.0f, math::Tolerance<float>());
}

TEST_F(TestLinearTimeInvariantShapes, fourth_order_single_output_steps)
{
    auto lti = MakeChain<4, 1, 1>();

    math::Vector<float, 4> x{};
    for (std::size_t i = 0; i < 4; ++i)
        x.at(i, 0) = static_cast<float>(i + 1);

    math::Vector<float, 1> u{};
    u.at(0, 0) = 2.0f;

    const auto next = lti.Step(x, u);
    EXPECT_NEAR(next.at(0, 0), 0.5f * 1.0f + 2.0f + 2.0f, math::Tolerance<float>());
    EXPECT_NEAR(next.at(3, 0), 0.5f * 4.0f, math::Tolerance<float>());

    const auto y = lti.Output(x, u);
    EXPECT_NEAR(y.at(0, 0), 1.0f, math::Tolerance<float>());
}

TEST_F(TestLinearTimeInvariantShapes, two_input_shape_applies_both_channels)
{
    auto lti = MakeChain<2, 2, 2>();

    math::Vector<float, 2> x{};
    x.at(0, 0) = 1.0f;
    x.at(1, 0) = 1.0f;

    math::Vector<float, 2> u{};
    u.at(0, 0) = 1.0f;
    u.at(1, 0) = -1.0f;

    const auto next = lti.Step(x, u);
    EXPECT_NEAR(next.at(0, 0), 0.5f + 1.0f + 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(next.at(1, 0), 0.5f - 1.0f, math::Tolerance<float>());
}

TEST_F(TestLinearTimeInvariantShapes, factory_helpers_populate_output_matrices)
{
    math::SquareMatrix<float, 4> a{};
    for (std::size_t i = 0; i < 4; ++i)
        a.at(i, i) = 0.9f;

    math::Matrix<float, 4, 1> b{};
    b.at(0, 0) = 1.0f;

    const auto full = math::LinearTimeInvariant<float, 4, 1>::WithFullStateOutput(a, b);

    for (std::size_t i = 0; i < 4; ++i)
        EXPECT_NEAR(full.C.at(i, i), 1.0f, math::Tolerance<float>());

    math::Matrix<float, 4, 4> c{};
    c.at(0, 0) = 2.0f;

    const auto autonomous = math::LinearTimeInvariant<float, 4, 1>::Autonomous(a, c);

    EXPECT_NEAR(autonomous.C.at(0, 0), 2.0f, math::Tolerance<float>());
    EXPECT_NEAR(autonomous.B.at(0, 0), 0.0f, math::Tolerance<float>());
}
