#include "numerical/controllers/test_doubles/Tolerance.hpp"
#include "numerical/filters/passive/Fir.hpp"
#include "numerical/math/RecursiveBuffer.hpp"
#include "gtest/gtest.h"

namespace
{
    template<typename T>
    class TestFir
        : public ::testing::Test
    {
    public:
        static constexpr std::size_t Order = 3;
        math::RecursiveBuffer<T, Order> b_coeffs{};
        std::optional<filters::passive::Fir<T, Order>> filter;
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestFir, TestedTypes);
}

TYPED_TEST(TestFir, when_disabled_returns_input_unchanged)
{
    this->b_coeffs = { TypeParam(0.3f), TypeParam(0.2f), TypeParam(0.1f) };
    this->filter.emplace(this->b_coeffs);
    this->filter->Disable();

    TypeParam input = TypeParam(0.4f);
    EXPECT_EQ(math::ToFloat(this->filter->Filter(input)), math::ToFloat(input));
}

TYPED_TEST(TestFir, simple_moving_average_filter_check)
{
    float tolerance = controllers::GetTolerance<TypeParam>();

    // Coefficients for a 3-point moving average (each coefficient = 0.25)
    this->b_coeffs = { TypeParam(0.25f), TypeParam(0.25f), TypeParam(0.25f) };
    this->filter.emplace(this->b_coeffs);

    EXPECT_NEAR(math::ToFloat(this->filter->Filter(TypeParam(0.4f))), 0.1f, tolerance);
    EXPECT_NEAR(math::ToFloat(this->filter->Filter(TypeParam(0.6f))), 0.25f, tolerance);
    EXPECT_NEAR(math::ToFloat(this->filter->Filter(TypeParam(0.2f))), 0.3f, tolerance);
}

TYPED_TEST(TestFir, reset_clears_filter_state)
{
    float tolerance = controllers::GetTolerance<TypeParam>();

    this->b_coeffs = { TypeParam(0.3f), TypeParam(0.2f), TypeParam(0.1f) };
    this->filter.emplace(this->b_coeffs);

    this->filter->Filter(TypeParam(0.4f));
    this->filter->Filter(TypeParam(0.6f));

    this->filter->Reset();

    // After reset, the first output should be the same as if we just started
    EXPECT_NEAR(math::ToFloat(this->filter->Filter(TypeParam(0.4f))), 0.12f, tolerance);
}

TYPED_TEST(TestFir, enable_disable_toggle)
{
    float tolerance = controllers::GetTolerance<TypeParam>();

    this->b_coeffs = { TypeParam(0.3f), TypeParam(0.2f), TypeParam(0.1f) };
    this->filter.emplace(this->b_coeffs);

    TypeParam input = TypeParam(0.4f);

    this->filter->Disable();
    EXPECT_EQ(math::ToFloat(this->filter->Filter(input)), math::ToFloat(input));

    this->filter->Enable();
    EXPECT_NEAR(math::ToFloat(this->filter->Filter(input)), 0.12f, tolerance);
}

TYPED_TEST(TestFir, weighted_average_filter_check)
{
    float tolerance = controllers::GetTolerance<TypeParam>();

    // Weighted average with decreasing weights
    this->b_coeffs = { TypeParam(0.3f), TypeParam(0.2f), TypeParam(0.1f) };
    this->filter.emplace(this->b_coeffs);

    EXPECT_NEAR(math::ToFloat(this->filter->Filter(TypeParam(0.6f))), 0.18f, tolerance);
    EXPECT_NEAR(math::ToFloat(this->filter->Filter(TypeParam(0.4f))), 0.24f, tolerance);
    EXPECT_NEAR(math::ToFloat(this->filter->Filter(TypeParam(0.5f))), 0.29f, tolerance);
}

TYPED_TEST(TestFir, zero_coefficients_produce_zero_output)
{
    this->b_coeffs = { TypeParam(0.0f), TypeParam(0.0f), TypeParam(0.0f) };
    this->filter.emplace(this->b_coeffs);

    EXPECT_EQ(math::ToFloat(this->filter->Filter(TypeParam(0.4f))), 0.0f);
    EXPECT_EQ(math::ToFloat(this->filter->Filter(TypeParam(-0.4f))), 0.0f);
}
