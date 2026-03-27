#include "numerical/solvers/DurandKerner.hpp"
#include <array>
#include <gtest/gtest.h>

namespace
{
    template<typename T>
    class TestDurandKerner : public ::testing::Test
    {
    protected:
        solvers::DurandKerner<T, 10> solver;
    };

    using TestTypes = ::testing::Types<float, double>;
    TYPED_TEST_SUITE(TestDurandKerner, TestTypes);
}

TYPED_TEST(TestDurandKerner, finds_roots_of_linear_polynomial)
{
    std::array<TypeParam, 2> coefficients = { TypeParam(2.0), TypeParam(4.0) };

    auto roots = this->solver.Solve(coefficients);

    ASSERT_EQ(roots.size(), 1u);
    EXPECT_NEAR(roots[0].real(), -2.0, 1e-4);
    EXPECT_NEAR(roots[0].imag(), 0.0, 1e-4);
}

TYPED_TEST(TestDurandKerner, finds_real_roots_of_quadratic)
{
    std::array<TypeParam, 3> coefficients = { TypeParam(1.0), TypeParam(-3.0), TypeParam(2.0) };

    auto roots = this->solver.Solve(coefficients);

    ASSERT_EQ(roots.size(), 2u);
    EXPECT_NEAR(roots[0].real(), 1.0, 1e-4);
    EXPECT_NEAR(roots[0].imag(), 0.0, 1e-4);
    EXPECT_NEAR(roots[1].real(), 2.0, 1e-4);
    EXPECT_NEAR(roots[1].imag(), 0.0, 1e-4);
}

TYPED_TEST(TestDurandKerner, finds_complex_roots_of_quadratic)
{
    std::array<TypeParam, 3> coefficients = { TypeParam(1.0), TypeParam(0.0), TypeParam(1.0) };

    auto roots = this->solver.Solve(coefficients);

    ASSERT_EQ(roots.size(), 2u);
    EXPECT_NEAR(roots[0].real(), 0.0, 1e-4);
    EXPECT_NEAR(std::abs(roots[0].imag()), 1.0, 1e-4);
    EXPECT_NEAR(roots[1].real(), 0.0, 1e-4);
    EXPECT_NEAR(std::abs(roots[1].imag()), 1.0, 1e-4);
}

TYPED_TEST(TestDurandKerner, finds_roots_of_cubic)
{
    std::array<TypeParam, 4> coefficients = { TypeParam(1.0), TypeParam(-6.0), TypeParam(11.0), TypeParam(-6.0) };

    auto roots = this->solver.Solve(coefficients);

    ASSERT_EQ(roots.size(), 3u);
    EXPECT_NEAR(roots[0].real(), 1.0, 1e-3);
    EXPECT_NEAR(roots[0].imag(), 0.0, 1e-3);
    EXPECT_NEAR(roots[1].real(), 2.0, 1e-3);
    EXPECT_NEAR(roots[1].imag(), 0.0, 1e-3);
    EXPECT_NEAR(roots[2].real(), 3.0, 1e-3);
    EXPECT_NEAR(roots[2].imag(), 0.0, 1e-3);
}

TYPED_TEST(TestDurandKerner, finds_roots_of_quartic)
{
    std::array<TypeParam, 5> coefficients = { TypeParam(1.0), TypeParam(-10.0), TypeParam(35.0), TypeParam(-50.0), TypeParam(24.0) };

    auto roots = this->solver.Solve(coefficients);

    ASSERT_EQ(roots.size(), 4u);
    EXPECT_NEAR(roots[0].real(), 1.0, 1e-2);
    EXPECT_NEAR(roots[1].real(), 2.0, 1e-2);
    EXPECT_NEAR(roots[2].real(), 3.0, 1e-2);
    EXPECT_NEAR(roots[3].real(), 4.0, 1e-2);
}

TYPED_TEST(TestDurandKerner, finds_repeated_roots)
{
    std::array<TypeParam, 3> coefficients = { TypeParam(1.0), TypeParam(-2.0), TypeParam(1.0) };

    auto roots = this->solver.Solve(coefficients);

    ASSERT_EQ(roots.size(), 2u);
    EXPECT_NEAR(roots[0].real(), 1.0, 1e-3);
    EXPECT_NEAR(roots[1].real(), 1.0, 1e-3);
}

TYPED_TEST(TestDurandKerner, returns_empty_for_constant_polynomial)
{
    std::array<TypeParam, 1> coefficients = { TypeParam(5.0) };

    auto roots = this->solver.Solve(coefficients);

    EXPECT_EQ(roots.size(), 0u);
}

TYPED_TEST(TestDurandKerner, finds_roots_of_second_order_system_polynomial)
{
    TypeParam wn(2.0);
    TypeParam zeta(0.5);
    std::array<TypeParam, 3> coefficients = { TypeParam(1.0), TypeParam(2.0) * zeta * wn, wn * wn };

    auto roots = this->solver.Solve(coefficients);

    ASSERT_EQ(roots.size(), 2u);
    EXPECT_NEAR(roots[0].real(), double(-zeta * wn), 1e-3);
    EXPECT_NEAR(roots[1].real(), double(-zeta * wn), 1e-3);
    EXPECT_NEAR(std::abs(roots[0].imag()), double(wn * std::sqrt(TypeParam(1.0) - zeta * zeta)), 1e-3);
}
