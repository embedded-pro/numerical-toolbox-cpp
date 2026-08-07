#include "numerical/filters/passive/MovingAverage.hpp"
#include "numerical/filters/passive/SavitzkyGolayFilter.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gtest/gtest.h"
#include <cmath>
#include <numeric>

namespace
{
    class TestSavitzkyGolayFilter
        : public ::testing::Test
    {
    public:
        filters::passive::SavitzkyGolayFilter<float, 5, 2, 0> sg{};
    };
}

TEST_F(TestSavitzkyGolayFilter, coefficients_sum_to_one)
{
    constexpr auto coeffs = filters::passive::SavitzkyGolayFilter<float, 5, 2, 0>::Coefficients();
    float sum{};
    for (float c : coeffs)
        sum += c;
    EXPECT_NEAR(sum, 1.0f, math::Tolerance<float>());
}

TEST_F(TestSavitzkyGolayFilter, known_5pt_quadratic_kernel)
{
    constexpr auto coeffs = filters::passive::SavitzkyGolayFilter<float, 5, 2, 0>::Coefficients();
    constexpr std::array<float, 5> expected{ -3.0f / 35.0f, 12.0f / 35.0f, 17.0f / 35.0f, 12.0f / 35.0f, -3.0f / 35.0f };
    constexpr float tol = 1e-5f;
    for (std::size_t i = 0; i < 5; ++i)
        EXPECT_NEAR(coeffs[i], expected[i], tol);
}

TEST_F(TestSavitzkyGolayFilter, fits_polynomial_exactly)
{
    constexpr float tol = 2e-4f;
    for (int n = 0; n < 20; ++n)
    {
        float x = static_cast<float>(n);
        float poly = 3.0f * x * x - 2.0f * x + 1.0f;
        float out = sg.Filter(poly);
        if (n >= 4)
            EXPECT_NEAR(out, 3.0f * (x - 2.0f) * (x - 2.0f) - 2.0f * (x - 2.0f) + 1.0f, tol);
    }
}

TEST_F(TestSavitzkyGolayFilter, preserves_peak_height)
{
    filters::passive::MovingAverage<float, 5> ma{};

    constexpr std::array<float, 9> signal{
        0.0f, 0.1f, 0.3f, 0.7f, 1.0f, 0.7f, 0.3f, 0.1f, 0.0f
    };

    float sgPeak{};
    float maPeak{};

    for (std::size_t i = 0; i < signal.size(); ++i)
    {
        float sgOut = sg.Filter(signal[i]);
        float maOut = ma.Filter(signal[i]);
        if (sgOut > sgPeak)
            sgPeak = sgOut;
        if (maOut > maPeak)
            maPeak = maOut;
    }

    EXPECT_GT(sgPeak, maPeak);
}

TEST_F(TestSavitzkyGolayFilter, first_derivative_of_ramp_is_constant)
{
    constexpr float slope = 2.5f;
    constexpr float tol = 1e-3f;

    filters::passive::SavitzkyGolayFilter<float, 5, 2, 1> deriv{};

    for (int n = 0; n < 20; ++n)
        deriv.Filter(slope * static_cast<float>(n));

    for (int n = 20; n < 25; ++n)
    {
        float out = deriv.Filter(slope * static_cast<float>(n));
        EXPECT_NEAR(out, slope, tol);
    }
}

TEST_F(TestSavitzkyGolayFilter, reset_clears_line)
{
    constexpr float tol = 1e-5f;

    for (int n = 0; n < 10; ++n)
        sg.Filter(static_cast<float>(n));

    sg.Reset(0.0f);

    constexpr float x = 1.0f;
    float out = sg.Filter(x);

    constexpr auto coeffs = filters::passive::SavitzkyGolayFilter<float, 5, 2, 0>::Coefficients();
    float expected = coeffs[4] * x;

    EXPECT_NEAR(out, expected, tol);
}

TEST_F(TestSavitzkyGolayFilter, smoothing_kernel_is_symmetric)
{
    constexpr auto coeffs = filters::passive::SavitzkyGolayFilter<float, 5, 2, 0>::Coefficients();
    constexpr float tol = 1e-6f;
    for (std::size_t i = 0; i < 5 / 2; ++i)
        EXPECT_NEAR(coeffs[i], coeffs[4 - i], tol);
}

TEST_F(TestSavitzkyGolayFilter, derivative_kernel_is_antisymmetric)
{
    constexpr auto coeffs = filters::passive::SavitzkyGolayFilter<float, 5, 2, 1>::Coefficients();
    constexpr float tol = 1e-6f;
    for (std::size_t i = 0; i < 5 / 2; ++i)
        EXPECT_NEAR(coeffs[i], -coeffs[4 - i], tol);
    EXPECT_NEAR(coeffs[2], 0.0f, tol);
}

TEST_F(TestSavitzkyGolayFilter, derivative_kernel_sums_to_zero)
{
    constexpr auto coeffs = filters::passive::SavitzkyGolayFilter<float, 5, 2, 1>::Coefficients();
    float sum{};
    for (float c : coeffs)
        sum += c;
    EXPECT_NEAR(sum, 0.0f, math::Tolerance<float>());
}

TEST_F(TestSavitzkyGolayFilter, constant_signal_steady_state_dc_gain)
{
    constexpr float value = 7.5f;
    filters::passive::SavitzkyGolayFilter<float, 5, 2, 0> f{ value };
    float out = f.Filter(value);
    EXPECT_NEAR(out, value, math::Tolerance<float>());
}

TEST_F(TestSavitzkyGolayFilter, second_derivative_of_quadratic_recovers_curvature)
{
    constexpr float a = 3.0f;
    constexpr float tol = 2e-4f;
    filters::passive::SavitzkyGolayFilter<float, 5, 2, 2> d2{};

    for (int n = 0; n < 20; ++n)
    {
        float x = static_cast<float>(n);
        float out = d2.Filter(a * x * x + 5.0f * x - 2.0f);
        if (n >= 4)
            EXPECT_NEAR(out, a, tol);
    }
}

TEST_F(TestSavitzkyGolayFilter, reset_to_nonzero_fills_buffer)
{
    constexpr float value = 4.0f;
    constexpr float tol = 1e-5f;

    for (int n = 0; n < 10; ++n)
        sg.Filter(static_cast<float>(n) * 100.0f);

    sg.Reset(value);

    float out = sg.Filter(value);
    EXPECT_NEAR(out, value, tol);
}

TEST_F(TestSavitzkyGolayFilter, seven_point_cubic_kernel_matches_reference)
{
    constexpr auto coeffs = filters::passive::SavitzkyGolayFilter<float, 7, 3, 0>::Coefficients();
    constexpr std::array<float, 7> expected{
        -2.0f / 21.0f,
        1.0f / 7.0f,
        2.0f / 7.0f,
        1.0f / 3.0f,
        2.0f / 7.0f,
        1.0f / 7.0f,
        -2.0f / 21.0f
    };
    constexpr float tol = 1e-5f;
    for (std::size_t i = 0; i < 7; ++i)
        EXPECT_NEAR(coeffs[i], expected[i], tol);
}

TEST_F(TestSavitzkyGolayFilter, seven_point_cubic_fits_cubic_polynomial_exactly)
{
    constexpr float tol = 1e-1f;
    filters::passive::SavitzkyGolayFilter<float, 7, 3, 0> f{};

    for (int n = 0; n < 30; ++n)
    {
        float x = static_cast<float>(n);
        float poly = x * x * x + 2.0f * x + 5.0f;
        float out = f.Filter(poly);
        if (n >= 6)
        {
            float cx = x - 3.0f;
            float ref = cx * cx * cx + 2.0f * cx + 5.0f;
            EXPECT_NEAR(out, ref, tol);
        }
    }
}

TEST_F(TestSavitzkyGolayFilter, determinism_two_instances_agree)
{
    filters::passive::SavitzkyGolayFilter<float, 5, 2, 0> a{};
    filters::passive::SavitzkyGolayFilter<float, 5, 2, 0> b{};

    for (int n = 0; n < 15; ++n)
    {
        float x = static_cast<float>(n) * 1.3f + 0.7f;
        EXPECT_FLOAT_EQ(a.Filter(x), b.Filter(x));
    }
}

TEST_F(TestSavitzkyGolayFilter, reset_restores_fresh_instance_behaviour)
{
    filters::passive::SavitzkyGolayFilter<float, 5, 2, 0> fresh{};

    for (int n = 0; n < 12; ++n)
        sg.Filter(static_cast<float>(n) * 99.0f);

    sg.Reset();

    for (int n = 0; n < 8; ++n)
    {
        float x = static_cast<float>(n) * 3.0f;
        EXPECT_FLOAT_EQ(sg.Filter(x), fresh.Filter(x));
    }
}

TEST_F(TestSavitzkyGolayFilter, derivative_of_constant_is_zero)
{
    constexpr float tol = 1e-5f;
    constexpr float value = 5.0f;
    filters::passive::SavitzkyGolayFilter<float, 5, 2, 1> deriv{ value };

    for (int n = 0; n < 10; ++n)
    {
        float out = deriv.Filter(value);
        EXPECT_NEAR(out, 0.0f, tol);
    }
}
