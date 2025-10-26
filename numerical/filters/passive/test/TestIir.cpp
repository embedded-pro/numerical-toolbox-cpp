#include "numerical/filters/passive/Iir.hpp"
#include "numerical/math/RecursiveBuffer.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gtest/gtest.h"

namespace
{
    template<typename T>
    class TestIir
        : public ::testing::Test
    {
    public:
        static constexpr std::size_t Order = 2;
        math::RecursiveBuffer<T, Order> b_coeffs{};
        math::RecursiveBuffer<T, Order> a_coeffs{};
        std::optional<filters::passive::Iir<T, Order, Order>> filter;
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestIir, TestedTypes);
}

TYPED_TEST(TestIir, when_disabled_returns_input_unchanged)
{
    this->b_coeffs = { TypeParam(0.9f), TypeParam(0.0f) };
    this->a_coeffs = { TypeParam(0.9f), TypeParam(0.0f) };

    this->filter.emplace(this->b_coeffs, this->a_coeffs);
    this->filter->Disable();

    TypeParam input = TypeParam(0.5f);
    EXPECT_EQ(math::ToFloat(this->filter->Filter(input)), math::ToFloat(input));
}

TYPED_TEST(TestIir, first_order_lowpass_filter_check)
{
    float tolerance = math::Tolerance<TypeParam>();

    this->b_coeffs = { TypeParam(0.1f), TypeParam(0.0f) };
    this->a_coeffs = { TypeParam(0.5f), TypeParam(0.4f) };

    this->filter.emplace(this->b_coeffs, this->a_coeffs);

    EXPECT_NEAR(math::ToFloat(this->filter->Filter(TypeParam(0.5f))), 0.05f, tolerance);
    EXPECT_NEAR(math::ToFloat(this->filter->Filter(TypeParam(0.5f))), 0.075f, tolerance);
    EXPECT_NEAR(math::ToFloat(this->filter->Filter(TypeParam(0.5f))), 0.1075f, tolerance);
}

TYPED_TEST(TestIir, reset_clears_filter_state)
{
    float tolerance = math::Tolerance<TypeParam>();

    this->b_coeffs = { TypeParam(0.1f), TypeParam(0.0f) };
    this->a_coeffs = { TypeParam(0.5f), TypeParam(0.4f) };

    this->filter.emplace(this->b_coeffs, this->a_coeffs);

    this->filter->Filter(TypeParam(0.5f));
    this->filter->Filter(TypeParam(0.5f));

    this->filter->Reset();

    EXPECT_NEAR(math::ToFloat(this->filter->Filter(TypeParam(0.5f))), 0.05f, tolerance);
}

TYPED_TEST(TestIir, enable_disable_toggle)
{
    float tolerance = math::Tolerance<TypeParam>();

    this->b_coeffs = { TypeParam(0.1f), TypeParam(0.0f) };
    this->a_coeffs = { TypeParam(0.5f), TypeParam(0.4f) };

    this->filter.emplace(this->b_coeffs, this->a_coeffs);

    TypeParam input = TypeParam(0.5f);

    this->filter->Disable();
    EXPECT_EQ(math::ToFloat(this->filter->Filter(input)), math::ToFloat(input));

    this->filter->Enable();
    EXPECT_NEAR(math::ToFloat(this->filter->Filter(input)), 0.05f, tolerance);
}

TYPED_TEST(TestIir, second_order_filter_check)
{
    float tolerance = math::Tolerance<TypeParam>();

    this->b_coeffs = { TypeParam(0.1f), TypeParam(0.2f) };
    this->a_coeffs = { TypeParam(0.5f), TypeParam(-0.1f) };

    this->filter.emplace(this->b_coeffs, this->a_coeffs);

    EXPECT_NEAR(math::ToFloat(this->filter->Filter(TypeParam(0.5f))), 0.05f, tolerance);
    EXPECT_NEAR(math::ToFloat(this->filter->Filter(TypeParam(0.0f))), 0.125f, tolerance);
    EXPECT_NEAR(math::ToFloat(this->filter->Filter(TypeParam(0.0f))), 0.0575f, tolerance);
}

TYPED_TEST(TestIir, zero_coefficients_produce_zero_output)
{
    this->b_coeffs = { TypeParam(0.0f), TypeParam(0.0f) };
    this->a_coeffs = { TypeParam(0.5f), TypeParam(0.0f) };

    this->filter.emplace(this->b_coeffs, this->a_coeffs);

    EXPECT_EQ(math::ToFloat(this->filter->Filter(TypeParam(0.5f))), 0.0f);
    EXPECT_EQ(math::ToFloat(this->filter->Filter(TypeParam(-0.5f))), 0.0f);
}
