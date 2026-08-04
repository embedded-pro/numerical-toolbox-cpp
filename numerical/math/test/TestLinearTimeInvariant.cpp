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
            { 0.0f, 0.9f }
        },
        math::Matrix<float, 2, 1>{
            { 0.0f },
            { 0.1f }
        });

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
            { 0.0f, 0.0f, 0.7f }
        },
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
            { 0.0f, 0.8f }
        },
        math::Matrix<float, 2, 2>{
            { 0.1f, 0.0f },
            { 0.0f, 0.1f }
        });

    math::Vector<float, 2> x{ { 1.0f }, { 0.5f } };
    math::Vector<float, 2> u{ { 1.0f }, { 2.0f } };

    auto xNext = lti.Step(x, u);
    EXPECT_NEAR(xNext.at(0, 0), 0.9f * 1.0f + 0.1f * 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(xNext.at(1, 0), 0.8f * 0.5f + 0.1f * 2.0f, math::Tolerance<float>());
}
