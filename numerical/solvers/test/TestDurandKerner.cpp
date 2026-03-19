#include "numerical/solvers/DurandKerner.hpp"
#include <array>
#include <gmock/gmock.h>

namespace
{
    TEST(TestDurandKerner, finds_roots_of_linear_polynomial)
    {
        std::array<float, 2> coefficients = { 2.0f, 4.0f };
        solvers::DurandKerner<float, 10> solver;

        auto roots = solver.Solve(infra::MemoryRange<const float>(coefficients));

        ASSERT_EQ(roots.size(), 1u);
        EXPECT_NEAR(roots[0].real(), -2.0f, 1e-4f);
        EXPECT_NEAR(roots[0].imag(), 0.0f, 1e-4f);
    }

    TEST(TestDurandKerner, finds_real_roots_of_quadratic)
    {
        std::array<float, 3> coefficients = { 1.0f, -3.0f, 2.0f };
        solvers::DurandKerner<float, 10> solver;

        auto roots = solver.Solve(infra::MemoryRange<const float>(coefficients));

        ASSERT_EQ(roots.size(), 2u);
        EXPECT_NEAR(roots[0].real(), 1.0f, 1e-4f);
        EXPECT_NEAR(roots[0].imag(), 0.0f, 1e-4f);
        EXPECT_NEAR(roots[1].real(), 2.0f, 1e-4f);
        EXPECT_NEAR(roots[1].imag(), 0.0f, 1e-4f);
    }

    TEST(TestDurandKerner, finds_complex_roots_of_quadratic)
    {
        std::array<float, 3> coefficients = { 1.0f, 0.0f, 1.0f };
        solvers::DurandKerner<float, 10> solver;

        auto roots = solver.Solve(infra::MemoryRange<const float>(coefficients));

        ASSERT_EQ(roots.size(), 2u);
        EXPECT_NEAR(roots[0].real(), 0.0f, 1e-4f);
        EXPECT_NEAR(std::abs(roots[0].imag()), 1.0f, 1e-4f);
        EXPECT_NEAR(roots[1].real(), 0.0f, 1e-4f);
        EXPECT_NEAR(std::abs(roots[1].imag()), 1.0f, 1e-4f);
    }

    TEST(TestDurandKerner, finds_roots_of_cubic)
    {
        std::array<float, 4> coefficients = { 1.0f, -6.0f, 11.0f, -6.0f };
        solvers::DurandKerner<float, 10> solver;

        auto roots = solver.Solve(infra::MemoryRange<const float>(coefficients));

        ASSERT_EQ(roots.size(), 3u);
        EXPECT_NEAR(roots[0].real(), 1.0f, 1e-3f);
        EXPECT_NEAR(roots[0].imag(), 0.0f, 1e-3f);
        EXPECT_NEAR(roots[1].real(), 2.0f, 1e-3f);
        EXPECT_NEAR(roots[1].imag(), 0.0f, 1e-3f);
        EXPECT_NEAR(roots[2].real(), 3.0f, 1e-3f);
        EXPECT_NEAR(roots[2].imag(), 0.0f, 1e-3f);
    }

    TEST(TestDurandKerner, finds_roots_of_quartic)
    {
        std::array<float, 5> coefficients = { 1.0f, -10.0f, 35.0f, -50.0f, 24.0f };
        solvers::DurandKerner<float, 10> solver;

        auto roots = solver.Solve(infra::MemoryRange<const float>(coefficients));

        ASSERT_EQ(roots.size(), 4u);
        EXPECT_NEAR(roots[0].real(), 1.0f, 1e-2f);
        EXPECT_NEAR(roots[1].real(), 2.0f, 1e-2f);
        EXPECT_NEAR(roots[2].real(), 3.0f, 1e-2f);
        EXPECT_NEAR(roots[3].real(), 4.0f, 1e-2f);
    }

    TEST(TestDurandKerner, finds_repeated_roots)
    {
        std::array<float, 3> coefficients = { 1.0f, -2.0f, 1.0f };
        solvers::DurandKerner<float, 10> solver;

        auto roots = solver.Solve(infra::MemoryRange<const float>(coefficients));

        ASSERT_EQ(roots.size(), 2u);
        EXPECT_NEAR(roots[0].real(), 1.0f, 1e-3f);
        EXPECT_NEAR(roots[1].real(), 1.0f, 1e-3f);
    }

    TEST(TestDurandKerner, returns_empty_for_constant_polynomial)
    {
        std::array<float, 1> coefficients = { 5.0f };
        solvers::DurandKerner<float, 10> solver;

        auto roots = solver.Solve(infra::MemoryRange<const float>(coefficients));

        EXPECT_EQ(roots.size(), 0u);
    }

    TEST(TestDurandKerner, finds_roots_with_double_precision)
    {
        std::array<double, 3> coefficients = { 1.0, -3.0, 2.0 };
        solvers::DurandKerner<double, 10> solver;

        auto roots = solver.Solve(infra::MemoryRange<const double>(coefficients));

        ASSERT_EQ(roots.size(), 2u);
        EXPECT_NEAR(roots[0].real(), 1.0, 1e-10);
        EXPECT_NEAR(roots[1].real(), 2.0, 1e-10);
    }

    TEST(TestDurandKerner, finds_roots_of_second_order_system_polynomial)
    {
        float wn = 2.0f;
        float zeta = 0.5f;
        std::array<float, 3> coefficients = { 1.0f, 2.0f * zeta * wn, wn * wn };
        solvers::DurandKerner<float, 10> solver;

        auto roots = solver.Solve(infra::MemoryRange<const float>(coefficients));

        ASSERT_EQ(roots.size(), 2u);
        EXPECT_NEAR(roots[0].real(), -zeta * wn, 1e-3f);
        EXPECT_NEAR(roots[1].real(), -zeta * wn, 1e-3f);
        EXPECT_NEAR(std::abs(roots[0].imag()), wn * std::sqrt(1.0f - zeta * zeta), 1e-3f);
    }
}
