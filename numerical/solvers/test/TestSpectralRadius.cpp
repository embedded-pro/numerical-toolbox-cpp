#include "numerical/math/Tolerance.hpp"
#include "numerical/solvers/SpectralRadius.hpp"
#include <gtest/gtest.h>
#include <numbers>

namespace
{
    class TestSpectralRadius : public ::testing::Test
    {
    protected:
        solvers::SpectralRadius<float, 3> solver3;
        solvers::SpectralRadius<float, 2> solver2;
        solvers::SpectralRadius<float, 1> solver1;
    };
}

TEST_F(TestSpectralRadius, diagonal_matrix_rho_equals_largest_magnitude_diagonal)
{
    math::SquareMatrix<float, 3> A{ { 1.0f, 0.0f, 0.0f }, { 0.0f, 2.0f, 0.0f }, { 0.0f, 0.0f, 3.0f } };

    float rho = solver3.Compute(A);

    EXPECT_NEAR(rho, 3.0f, 1e-2f);
}

TEST_F(TestSpectralRadius, known_matrix_rho_matches_analytic_dominant_eigenvalue)
{
    math::SquareMatrix<float, 3> A{ { 3.0f, 1.0f, 0.0f }, { 0.0f, 2.0f, 0.0f }, { 0.0f, 0.0f, 1.0f } };

    float rho = solver3.Compute(A);

    EXPECT_NEAR(rho, 3.0f, 1e-2f);
}

TEST_F(TestSpectralRadius, complex_conjugate_pair_rho_equals_modulus)
{
    float r = 0.5f;
    float theta = std::numbers::pi_v<float> / 4.0f;
    math::SquareMatrix<float, 2> A{
        { r * std::cos(theta), -r * std::sin(theta) },
        { r * std::sin(theta), r * std::cos(theta) }
    };

    float rho = solver2.Compute(A);

    EXPECT_NEAR(rho, r, 1e-2f);
}

TEST_F(TestSpectralRadius, schur_stable_true_when_all_eigenvalues_inside_unit_disk)
{
    math::SquareMatrix<float, 3> A{ { 0.5f, 0.0f, 0.0f }, { 0.0f, 0.3f, 0.0f }, { 0.0f, 0.0f, 0.1f } };

    EXPECT_TRUE(solver3.IsSchurStable(A));
}

TEST_F(TestSpectralRadius, schur_stable_false_when_eigenvalue_outside_unit_disk)
{
    math::SquareMatrix<float, 3> A{ { 0.5f, 0.0f, 0.0f }, { 0.0f, 1.1f, 0.0f }, { 0.0f, 0.0f, 0.1f } };

    EXPECT_FALSE(solver3.IsSchurStable(A));
}

TEST_F(TestSpectralRadius, stability_margin_equals_one_minus_rho)
{
    math::SquareMatrix<float, 3> A{ { 0.5f, 0.0f, 0.0f }, { 0.0f, 0.3f, 0.0f }, { 0.0f, 0.0f, 0.1f } };

    float margin = solver3.StabilityMargin(A);

    EXPECT_NEAR(margin, 0.5f, 1e-2f);
}

TEST_F(TestSpectralRadius, scalar_size_one_rho_equals_absolute_value_of_entry)
{
    math::SquareMatrix<float, 1> A{ { -3.0f } };

    float rho = solver1.Compute(A);

    EXPECT_NEAR(rho, 3.0f, math::Tolerance<float>());
}
