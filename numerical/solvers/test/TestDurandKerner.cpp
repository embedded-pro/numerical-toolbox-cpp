#include "numerical/solvers/DurandKerner.hpp"
#include <array>
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    class TestDurandKerner : public ::testing::Test
    {
    protected:
        solvers::DurandKerner<float, 10> solver;
    };
}

TEST_F(TestDurandKerner, linear_polynomial_returns_single_exact_root)
{
    std::array<float, 2> coefficients{ 2.0f, 4.0f };

    auto roots = solver.Solve(coefficients);

    ASSERT_EQ(roots.size(), 1u);
    EXPECT_NEAR(roots[0].Real(), -2.0f, 1e-4f);
    EXPECT_NEAR(roots[0].Imaginary(), 0.0f, 1e-4f);
}

TEST_F(TestDurandKerner, real_quadratic_roots_match_factored_form)
{
    std::array<float, 3> coefficients{ 1.0f, -3.0f, 2.0f };

    auto roots = solver.Solve(coefficients);

    ASSERT_EQ(roots.size(), 2u);
    EXPECT_NEAR(roots[0].Real(), 1.0f, 1e-4f);
    EXPECT_NEAR(roots[0].Imaginary(), 0.0f, 1e-4f);
    EXPECT_NEAR(roots[1].Real(), 2.0f, 1e-4f);
    EXPECT_NEAR(roots[1].Imaginary(), 0.0f, 1e-4f);
}

TEST_F(TestDurandKerner, complex_conjugate_roots_have_unit_imaginary_magnitude)
{
    std::array<float, 3> coefficients{ 1.0f, 0.0f, 1.0f };

    auto roots = solver.Solve(coefficients);

    ASSERT_EQ(roots.size(), 2u);
    EXPECT_NEAR(roots[0].Real(), 0.0f, 1e-4f);
    EXPECT_NEAR(std::abs(roots[0].Imaginary()), 1.0f, 1e-4f);
    EXPECT_NEAR(roots[1].Real(), 0.0f, 1e-4f);
    EXPECT_NEAR(std::abs(roots[1].Imaginary()), 1.0f, 1e-4f);
}

TEST_F(TestDurandKerner, cubic_roots_match_factored_form_1_2_3)
{
    std::array<float, 4> coefficients{ 1.0f, -6.0f, 11.0f, -6.0f };

    auto roots = solver.Solve(coefficients);

    ASSERT_EQ(roots.size(), 3u);
    EXPECT_NEAR(roots[0].Real(), 1.0f, 1e-3f);
    EXPECT_NEAR(roots[0].Imaginary(), 0.0f, 1e-3f);
    EXPECT_NEAR(roots[1].Real(), 2.0f, 1e-3f);
    EXPECT_NEAR(roots[1].Imaginary(), 0.0f, 1e-3f);
    EXPECT_NEAR(roots[2].Real(), 3.0f, 1e-3f);
    EXPECT_NEAR(roots[2].Imaginary(), 0.0f, 1e-3f);
}

TEST_F(TestDurandKerner, quartic_roots_match_factored_form_1_2_3_4)
{
    std::array<float, 5> coefficients{ 1.0f, -10.0f, 35.0f, -50.0f, 24.0f };

    auto roots = solver.Solve(coefficients);

    ASSERT_EQ(roots.size(), 4u);
    EXPECT_NEAR(roots[0].Real(), 1.0f, 1e-2f);
    EXPECT_NEAR(roots[0].Imaginary(), 0.0f, 1e-2f);
    EXPECT_NEAR(roots[1].Real(), 2.0f, 1e-2f);
    EXPECT_NEAR(roots[1].Imaginary(), 0.0f, 1e-2f);
    EXPECT_NEAR(roots[2].Real(), 3.0f, 1e-2f);
    EXPECT_NEAR(roots[2].Imaginary(), 0.0f, 1e-2f);
    EXPECT_NEAR(roots[3].Real(), 4.0f, 1e-2f);
    EXPECT_NEAR(roots[3].Imaginary(), 0.0f, 1e-2f);
}

TEST_F(TestDurandKerner, repeated_root_both_approximations_converge_to_same_value)
{
    std::array<float, 3> coefficients{ 1.0f, -2.0f, 1.0f };

    auto roots = solver.Solve(coefficients);

    ASSERT_EQ(roots.size(), 2u);
    EXPECT_NEAR(roots[0].Real(), 1.0f, 1e-3f);
    EXPECT_NEAR(roots[1].Real(), 1.0f, 1e-3f);
}

TEST_F(TestDurandKerner, constant_polynomial_returns_empty)
{
    std::array<float, 1> coefficients{ 5.0f };

    auto roots = solver.Solve(coefficients);

    EXPECT_EQ(roots.size(), 0u);
}

TEST_F(TestDurandKerner, second_order_control_system_roots_match_analytic_poles)
{
    float wn = 2.0f;
    float zeta = 0.5f;
    std::array<float, 3> coefficients{ 1.0f, 2.0f * zeta * wn, wn * wn };

    auto roots = solver.Solve(coefficients);

    ASSERT_EQ(roots.size(), 2u);
    float expectedReal = -zeta * wn;
    float expectedImag = wn * std::sqrt(1.0f - zeta * zeta);
    EXPECT_NEAR(roots[0].Real(), expectedReal, 1e-3f);
    EXPECT_NEAR(roots[1].Real(), expectedReal, 1e-3f);
    EXPECT_NEAR(std::abs(roots[0].Imaginary()), expectedImag, 1e-3f);
}

TEST_F(TestDurandKerner, each_root_satisfies_polynomial_residual_near_zero)
{
    std::array<float, 4> coefficients{ 1.0f, -6.0f, 11.0f, -6.0f };

    auto roots = solver.Solve(coefficients);

    ASSERT_EQ(roots.size(), 3u);
    for (std::size_t i = 0; i < roots.size(); ++i)
    {
        float re = roots[i].Real();
        float im = roots[i].Imaginary();
        float re2 = re * re - im * im;
        float im2 = 2.0f * re * im;
        float re3 = re2 * re - im2 * im;
        float im3 = re2 * im + im2 * re;
        float pRe = re3 - 6.0f * re2 + 11.0f * re - 6.0f;
        float pIm = im3 - 6.0f * im2 + 11.0f * im;
        EXPECT_NEAR(pRe, 0.0f, 1e-2f);
        EXPECT_NEAR(pIm, 0.0f, 1e-2f);
    }
}

TEST_F(TestDurandKerner, same_input_produces_identical_real_parts_determinism)
{
    std::array<float, 4> coefficients{ 1.0f, -6.0f, 11.0f, -6.0f };

    auto roots1 = solver.Solve(coefficients);
    auto roots2 = solver.Solve(coefficients);

    ASSERT_EQ(roots1.size(), roots2.size());
    for (std::size_t i = 0; i < roots1.size(); ++i)
        EXPECT_FLOAT_EQ(roots1[i].Real(), roots2[i].Real());
}
