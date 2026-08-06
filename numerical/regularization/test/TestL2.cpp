#include "numerical/math/Tolerance.hpp"
#include "numerical/regularization/L2.hpp"
#include <gtest/gtest.h>
#include <random>

namespace
{
    class TestL2
        : public ::testing::Test
    {
    protected:
        static constexpr std::size_t Size = 4;
        using Vec = math::Vector<float, Size>;

        regularization::L2<float, Size> reg{ 0.1f };
    };
}

TEST_F(TestL2, calculate_returns_zero_for_zero_vector)
{
    Vec params{};
    EXPECT_NEAR(reg.Calculate(params), 0.0f, math::Tolerance<float>());
}

TEST_F(TestL2, calculate_penalty_equals_lambda_half_sum_of_squares)
{
    Vec params{ 1.0f, 2.0f, 3.0f, 4.0f };
    const float expected = 0.1f * (1.0f + 4.0f + 9.0f + 16.0f) / 2.0f;
    EXPECT_NEAR(reg.Calculate(params), expected, math::Tolerance<float>());
}

TEST_F(TestL2, calculate_penalty_is_sign_invariant)
{
    Vec pos{ 1.0f, 2.0f, 3.0f, 4.0f };
    Vec neg{ -1.0f, -2.0f, -3.0f, -4.0f };
    EXPECT_NEAR(reg.Calculate(pos), reg.Calculate(neg), math::Tolerance<float>());
}

TEST_F(TestL2, calculate_lambda_zero_yields_zero_penalty)
{
    regularization::L2<float, Size> zeroReg{ 0.0f };
    Vec params{ 1.0f, 2.0f, 3.0f, 4.0f };
    EXPECT_NEAR(zeroReg.Calculate(params), 0.0f, math::Tolerance<float>());
}

TEST_F(TestL2, calculate_large_lambda_scales_penalty)
{
    regularization::L2<float, Size> bigReg{ 1000.0f };
    Vec params{ 1.0f, 0.0f, 0.0f, 0.0f };
    const float expected = 1000.0f * 1.0f / 2.0f;
    EXPECT_NEAR(bigReg.Calculate(params), expected, math::Tolerance<float>());
}

TEST_F(TestL2, calculate_scales_quadratically_with_coefficient_magnitude)
{
    Vec w{ 1.0f, 2.0f, 3.0f, 4.0f };
    Vec w2{ 2.0f, 4.0f, 6.0f, 8.0f };
    EXPECT_NEAR(reg.Calculate(w2), 4.0f * reg.Calculate(w), math::Tolerance<float>());
}

TEST_F(TestL2, gradient_returns_zero_for_zero_vector)
{
    Vec params{};
    auto gradient = reg.Gradient(params);
    for (std::size_t i = 0; i < Size; ++i)
        EXPECT_NEAR(gradient[i], 0.0f, math::Tolerance<float>());
}

TEST_F(TestL2, gradient_equals_lambda_times_parameters)
{
    Vec params{ 1.0f, -2.0f, 3.0f, -4.0f };
    auto gradient = reg.Gradient(params);
    for (std::size_t i = 0; i < Size; ++i)
        EXPECT_NEAR(gradient[i], 0.1f * params[i], math::Tolerance<float>());
}

TEST_F(TestL2, gradient_lambda_zero_is_zero_vector)
{
    regularization::L2<float, Size> zeroReg{ 0.0f };
    Vec params{ 1.0f, -2.0f, 3.0f, -4.0f };
    auto gradient = zeroReg.Gradient(params);
    for (std::size_t i = 0; i < Size; ++i)
        EXPECT_NEAR(gradient[i], 0.0f, math::Tolerance<float>());
}

TEST_F(TestL2, gradient_is_directional_derivative_of_penalty)
{
    Vec params{ 1.0f, 2.0f, 3.0f, 4.0f };
    const float epsilon = 1e-3f;
    auto gradient = reg.Gradient(params);

    for (std::size_t i = 0; i < Size; ++i)
    {
        Vec perturbed = params;
        perturbed[i] += epsilon;
        const float finiteDiff = (reg.Calculate(perturbed) - reg.Calculate(params)) / epsilon;
        EXPECT_NEAR(gradient[i], finiteDiff, 1e-2f);
    }
}

TEST_F(TestL2, shrinkage_identity_ridge_closed_form)
{
    const float lambda = 0.1f;
    Vec params{ 2.0f, 4.0f, 6.0f, 8.0f };
    auto gradient = reg.Gradient(params);
    const float lr = 1.0f / (1.0f + lambda);
    for (std::size_t i = 0; i < Size; ++i)
    {
        const float shrunken = params[i] / (1.0f + lambda);
        const float fromGradient = params[i] - lr * gradient[i];
        EXPECT_NEAR(fromGradient, shrunken, math::Tolerance<float>());
    }
}

TEST_F(TestL2, calculate_is_deterministic)
{
    Vec params{ 1.0f, -2.0f, 3.0f, -4.0f };
    const float first = reg.Calculate(params);
    const float second = reg.Calculate(params);
    EXPECT_FLOAT_EQ(first, second);
}

TEST_F(TestL2, gradient_is_deterministic)
{
    Vec params{ 1.0f, -2.0f, 3.0f, -4.0f };
    auto g1 = reg.Gradient(params);
    auto g2 = reg.Gradient(params);
    for (std::size_t i = 0; i < Size; ++i)
        EXPECT_FLOAT_EQ(g1[i], g2[i]);
}

TEST_F(TestL2, calculate_scaling_homogeneity_over_seeded_sweep)
{
    std::mt19937 rng{ 42u };
    std::uniform_real_distribution<float> dist{ -5.0f, 5.0f };
    const float alpha = 3.0f;

    for (int trial = 0; trial < 16; ++trial)
    {
        Vec w;
        Vec aw;
        for (std::size_t i = 0; i < Size; ++i)
        {
            w[i] = dist(rng);
            aw[i] = alpha * w[i];
        }
        EXPECT_NEAR(reg.Calculate(aw), alpha * alpha * reg.Calculate(w), math::Tolerance<float>());
    }
}
