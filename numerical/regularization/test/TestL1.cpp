#include "numerical/math/Tolerance.hpp"
#include "numerical/regularization/L1.hpp"
#include <gtest/gtest.h>

namespace
{
    class TestL1 : public ::testing::Test
    {
    protected:
        static constexpr std::size_t Size = 4;
        using Vec = math::Vector<float, Size>;

        regularization::L1<float, Size> reg{ 0.1f };
    };
}

TEST_F(TestL1, calculate_zero_vector_returns_zero)
{
    Vec params{};
    EXPECT_NEAR(reg.Calculate(params), 0.0f, math::Tolerance<float>());
}

TEST_F(TestL1, calculate_equals_lambda_times_l1_norm)
{
    Vec params{ 1.0f, -2.0f, 3.0f, -4.0f };
    float expected = 0.1f * (1.0f + 2.0f + 3.0f + 4.0f);
    EXPECT_NEAR(reg.Calculate(params), expected, math::Tolerance<float>());
}

TEST_F(TestL1, calculate_symmetric_in_sign)
{
    Vec pos{ 1.0f, 2.0f, 3.0f, 4.0f };
    Vec neg{ -1.0f, -2.0f, -3.0f, -4.0f };
    EXPECT_NEAR(reg.Calculate(pos), reg.Calculate(neg), math::Tolerance<float>());
}

TEST_F(TestL1, calculate_lambda_zero_returns_zero)
{
    regularization::L1<float, Size> zeroReg{ 0.0f };
    Vec params{ 1.0f, -2.0f, 3.0f, -4.0f };
    EXPECT_NEAR(zeroReg.Calculate(params), 0.0f, math::Tolerance<float>());
}

TEST_F(TestL1, calculate_large_lambda_scales_penalty)
{
    regularization::L1<float, Size> bigReg{ 1000.0f };
    Vec params{ 0.001f, 0.001f, 0.001f, 0.001f };
    float expected = 1000.0f * 4.0f * 0.001f;
    EXPECT_NEAR(bigReg.Calculate(params), expected, 1e-1f);
}

TEST_F(TestL1, gradient_positive_parameters_equals_lambda)
{
    Vec params{ 0.5f, 1.0f, 2.0f, 3.0f };
    auto gradient = reg.Gradient(params);
    for (std::size_t i = 0; i < Size; ++i)
        EXPECT_NEAR(gradient[i], 0.1f, math::Tolerance<float>());
}

TEST_F(TestL1, gradient_negative_parameters_equals_minus_lambda)
{
    Vec params{ -0.5f, -1.0f, -2.0f, -3.0f };
    auto gradient = reg.Gradient(params);
    for (std::size_t i = 0; i < Size; ++i)
        EXPECT_NEAR(gradient[i], -0.1f, math::Tolerance<float>());
}

TEST_F(TestL1, gradient_zero_parameter_is_zero)
{
    Vec params{};
    auto gradient = reg.Gradient(params);
    for (std::size_t i = 0; i < Size; ++i)
        EXPECT_NEAR(gradient[i], 0.0f, math::Tolerance<float>());
}

TEST_F(TestL1, gradient_mixed_signs_follow_signum)
{
    Vec params{ 1.0f, -1.0f, 0.0f, 2.0f };
    auto gradient = reg.Gradient(params);
    EXPECT_NEAR(gradient[0], 0.1f, math::Tolerance<float>());
    EXPECT_NEAR(gradient[1], -0.1f, math::Tolerance<float>());
    EXPECT_NEAR(gradient[2], 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(gradient[3], 0.1f, math::Tolerance<float>());
}

TEST_F(TestL1, gradient_magnitude_is_lambda_independent_of_magnitude)
{
    Vec small{ 1e-5f, 2e-5f, 3e-5f, 4e-5f };
    Vec large{ 1e3f, 2e3f, 3e3f, 4e3f };
    auto gSmall = reg.Gradient(small);
    auto gLarge = reg.Gradient(large);
    for (std::size_t i = 0; i < Size; ++i)
    {
        EXPECT_NEAR(gSmall[i], 0.1f, math::Tolerance<float>());
        EXPECT_NEAR(gLarge[i], 0.1f, math::Tolerance<float>());
    }
}

TEST_F(TestL1, calculate_deterministic_same_input_same_output)
{
    Vec params{ 0.3f, -0.7f, 1.2f, -0.9f };
    float first = reg.Calculate(params);
    float second = reg.Calculate(params);
    EXPECT_FLOAT_EQ(first, second);
}

TEST_F(TestL1, gradient_deterministic_same_input_same_output)
{
    Vec params{ 0.3f, -0.7f, 1.2f, -0.9f };
    auto g1 = reg.Gradient(params);
    auto g2 = reg.Gradient(params);
    for (std::size_t i = 0; i < Size; ++i)
        EXPECT_FLOAT_EQ(g1[i], g2[i]);
}

TEST_F(TestL1, penalty_equals_lambda_times_sum_of_absolute_values_property)
{
    std::array<float, 4> rawParams{ 0.3f, -0.7f, 1.2f, -0.9f };
    Vec params{ rawParams[0], rawParams[1], rawParams[2], rawParams[3] };
    float l1Norm = 0.0f;
    for (auto v : rawParams)
        l1Norm += (v < 0.0f ? -v : v);
    EXPECT_NEAR(reg.Calculate(params), 0.1f * l1Norm, math::Tolerance<float>());
}
