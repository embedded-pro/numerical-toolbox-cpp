#include "numerical/math/LinearTimeInvariant.hpp"
#include <gtest/gtest.h>

namespace
{
    template<typename T>
    class TestLinearTimeInvariant : public ::testing::Test
    {
    protected:
        static T Val(float f)
        {
            f = std::max(std::min(f, 0.9999f), -0.9999f);
            if constexpr (std::is_same_v<T, float>)
                return f;
            else
                return T(f);
        }

        static float ToF(T v)
        {
            if constexpr (std::is_same_v<T, float>)
                return v;
            else
                return v.ToFloat();
        }
    };

    using TestTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestLinearTimeInvariant, TestTypes);
}

TYPED_TEST(TestLinearTimeInvariant, default_construction_zeroes_A_B_C_D)
{
    math::LinearTimeInvariant<TypeParam, 2, 1> lti;

    for (std::size_t r = 0; r < 2; ++r)
        for (std::size_t c = 0; c < 2; ++c)
            EXPECT_NEAR(this->ToF(lti.A.at(r, c)), 0.0f, 1e-5f);

    for (std::size_t r = 0; r < 2; ++r)
        EXPECT_NEAR(this->ToF(lti.B.at(r, 0)), 0.0f, 1e-5f);
}

TYPED_TEST(TestLinearTimeInvariant, step_returns_A_x_plus_B_u)
{
    math::LinearTimeInvariant<TypeParam, 2, 1> lti;
    lti.A = math::SquareMatrix<TypeParam, 2>{
        { this->Val(0.9999f), this->Val(0.5f) },
        { this->Val(0.0f), this->Val(0.9f) }
    };
    lti.B = math::Matrix<TypeParam, 2, 1>{
        { this->Val(0.0f) },
        { this->Val(0.1f) }
    };

    math::Vector<TypeParam, 2> x{ { this->Val(0.5f) }, { this->Val(0.25f) } };
    math::Vector<TypeParam, 1> u{ { this->Val(0.1f) } };

    auto xNext = lti.Step(x, u);

    // x_next[0] = ~1.0*0.5 + 0.5*0.25 + 0.0*0.1 ~= 0.625
    // x_next[1] = 0.0*0.5 + 0.9*0.25 + 0.1*0.1 = 0.235
    EXPECT_NEAR(this->ToF(xNext.at(0, 0)), 0.625f, 1e-2f);
    EXPECT_NEAR(this->ToF(xNext.at(1, 0)), 0.235f, 1e-2f);
}

TYPED_TEST(TestLinearTimeInvariant, output_returns_C_x_plus_D_u)
{
    math::LinearTimeInvariant<TypeParam, 2, 1, 1> lti;
    lti.C = math::Matrix<TypeParam, 1, 2>{
        { TypeParam(0.9f), TypeParam(0.0f) }
    };
    lti.D = math::Matrix<TypeParam, 1, 1>{ { TypeParam(0.0f) } };

    math::Vector<TypeParam, 2> x{ { TypeParam(0.5f) }, { TypeParam(0.25f) } };
    math::Vector<TypeParam, 1> u{ { TypeParam(0.1f) } };

    auto y = lti.Output(x, u);

    // y = 0.9*0.5 + 0.0*0.25 = 0.45
    EXPECT_NEAR(this->ToF(y.at(0, 0)), 0.45f, 5e-3f);
}

TYPED_TEST(TestLinearTimeInvariant, with_full_state_output_sets_C_to_identity_and_D_to_zero)
{
    auto A = math::SquareMatrix<TypeParam, 2>{
        { TypeParam(0.9f), TypeParam(0.1f) },
        { TypeParam(0.0f), TypeParam(0.8f) }
    };
    auto B = math::Matrix<TypeParam, 2, 1>{
        { TypeParam(0.0f) },
        { TypeParam(0.1f) }
    };

    auto lti = math::LinearTimeInvariant<TypeParam, 2, 1>::WithFullStateOutput(A, B);

    // C should be identity
    EXPECT_NEAR(this->ToF(lti.C.at(0, 0)), 1.0f, 5e-3f);
    EXPECT_NEAR(this->ToF(lti.C.at(1, 1)), 1.0f, 5e-3f);
    EXPECT_NEAR(this->ToF(lti.C.at(0, 1)), 0.0f, 1e-5f);
    EXPECT_NEAR(this->ToF(lti.C.at(1, 0)), 0.0f, 1e-5f);

    // D should be zero
    EXPECT_NEAR(this->ToF(lti.D.at(0, 0)), 0.0f, 1e-5f);
}

TYPED_TEST(TestLinearTimeInvariant, autonomous_factory_zeroes_B)
{
    auto A = math::SquareMatrix<TypeParam, 2>{
        { TypeParam(0.9f), TypeParam(0.0f) },
        { TypeParam(0.0f), TypeParam(0.8f) }
    };
    auto C = math::Matrix<TypeParam, 1, 2>{
        { TypeParam(0.9f), TypeParam(0.0f) }
    };

    auto lti = math::LinearTimeInvariant<TypeParam, 2, 1, 1>::Autonomous(A, C);

    for (std::size_t r = 0; r < 2; ++r)
        EXPECT_NEAR(this->ToF(lti.B.at(r, 0)), 0.0f, 1e-5f);
}

TYPED_TEST(TestLinearTimeInvariant, step_called_twice_propagates_state_correctly)
{
    auto lti = math::LinearTimeInvariant<TypeParam, 2, 1>::WithFullStateOutput(
        math::SquareMatrix<TypeParam, 2>{
            { this->Val(0.9999f), this->Val(0.1f) },
            { this->Val(0.0f), this->Val(0.9f) } },
        math::Matrix<TypeParam, 2, 1>{
            { this->Val(0.0f) },
            { this->Val(0.1f) } });

    math::Vector<TypeParam, 2> x{ { TypeParam(0.0f) }, { TypeParam(0.0f) } };
    math::Vector<TypeParam, 1> u{ { TypeParam(0.5f) } };

    x = lti.Step(x, u);
    x = lti.Step(x, u);

    // After step 1: x = [0, 0.05]
    // After step 2: x = [0.005, 0.095]
    EXPECT_NEAR(this->ToF(x.at(0, 0)), 0.005f, 5e-3f);
    EXPECT_NEAR(this->ToF(x.at(1, 0)), 0.095f, 5e-3f);
}

namespace
{
    class TestLinearTimeInvariantFloat : public ::testing::Test
    {};
}

TEST_F(TestLinearTimeInvariantFloat, three_state_system_step_and_output)
{
    auto A = math::SquareMatrix<float, 3>{
        { 0.9f, 0.1f, 0.0f },
        { 0.0f, 0.8f, 0.1f },
        { 0.0f, 0.0f, 0.7f }
    };
    auto B = math::Matrix<float, 3, 1>{ { 0.0f }, { 0.0f }, { 0.1f } };

    auto lti = math::LinearTimeInvariant<float, 3, 1>::WithFullStateOutput(A, B);

    math::Vector<float, 3> x{ { 1.0f }, { 0.5f }, { 0.25f } };
    math::Vector<float, 1> u{ { 2.0f } };

    auto xNext = lti.Step(x, u);
    EXPECT_NEAR(xNext.at(0, 0), 0.9f * 1.0f + 0.1f * 0.5f, 1e-5f);
    EXPECT_NEAR(xNext.at(1, 0), 0.8f * 0.5f + 0.1f * 0.25f, 1e-5f);
    EXPECT_NEAR(xNext.at(2, 0), 0.7f * 0.25f + 0.1f * 2.0f, 1e-5f);

    // C = I, D = 0, so Output(x, u) == x
    auto y = lti.Output(x, u);
    EXPECT_NEAR(y.at(0, 0), x.at(0, 0), 5e-3f);
    EXPECT_NEAR(y.at(1, 0), x.at(1, 0), 5e-3f);
    EXPECT_NEAR(y.at(2, 0), x.at(2, 0), 5e-3f);
}

TEST_F(TestLinearTimeInvariantFloat, four_state_system_step)
{
    auto lti = math::LinearTimeInvariant<float, 4, 1>::WithFullStateOutput(
        math::SquareMatrix<float, 4>{
            { 0.9f, 0.1f, 0.0f, 0.0f },
            { 0.0f, 0.8f, 0.1f, 0.0f },
            { 0.0f, 0.0f, 0.7f, 0.1f },
            { 0.0f, 0.0f, 0.0f, 0.6f } },
        math::Matrix<float, 4, 1>{ { 0.0f }, { 0.0f }, { 0.0f }, { 0.1f } });

    math::Vector<float, 4> x{ { 1.0f }, { 0.0f }, { 0.0f }, { 0.0f } };
    math::Vector<float, 1> u{ { 1.0f } };

    auto xNext = lti.Step(x, u);
    EXPECT_NEAR(xNext.at(0, 0), 0.9f, 1e-5f);
    EXPECT_NEAR(xNext.at(3, 0), 0.1f, 1e-5f);
}

TEST_F(TestLinearTimeInvariantFloat, four_state_single_output_autonomous)
{
    auto A = math::SquareMatrix<float, 4>{
        { 0.9f, 0.0f, 0.0f, 0.0f },
        { 0.0f, 0.8f, 0.0f, 0.0f },
        { 0.0f, 0.0f, 0.7f, 0.0f },
        { 0.0f, 0.0f, 0.0f, 0.6f }
    };
    auto C = math::Matrix<float, 1, 4>{ { 1.0f, 0.0f, 0.0f, 0.0f } };

    auto lti = math::LinearTimeInvariant<float, 4, 1, 1>::Autonomous(A, C);

    for (std::size_t r = 0; r < 4; ++r)
        EXPECT_NEAR(lti.B.at(r, 0), 0.0f, 1e-5f);

    math::Vector<float, 4> x{ { 1.0f }, { 0.5f }, { 0.25f }, { 0.1f } };
    math::Vector<float, 1> u{};
    auto xNext = lti.Step(x, u);
    EXPECT_NEAR(xNext.at(0, 0), 0.9f, 1e-5f);
}

TEST_F(TestLinearTimeInvariantFloat, two_state_two_input_with_full_state_output)
{
    auto A = math::SquareMatrix<float, 2>{
        { 0.9f, 0.0f },
        { 0.0f, 0.8f }
    };
    auto B = math::Matrix<float, 2, 2>{
        { 0.1f, 0.0f },
        { 0.0f, 0.1f }
    };

    auto lti = math::LinearTimeInvariant<float, 2, 2>::WithFullStateOutput(A, B);

    // C must be identity
    EXPECT_NEAR(lti.C.at(0, 0), 1.0f, 5e-3f);
    EXPECT_NEAR(lti.C.at(1, 1), 1.0f, 5e-3f);
    EXPECT_NEAR(lti.C.at(0, 1), 0.0f, 5e-3f);
    EXPECT_NEAR(lti.C.at(1, 0), 0.0f, 5e-3f);

    math::Vector<float, 2> x{ { 1.0f }, { 0.5f } };
    math::Vector<float, 2> u{ { 1.0f }, { 2.0f } };

    auto xNext = lti.Step(x, u);
    EXPECT_NEAR(xNext.at(0, 0), 0.9f + 0.1f, 1e-5f);
    EXPECT_NEAR(xNext.at(1, 0), 0.4f + 0.2f, 1e-5f);
}
