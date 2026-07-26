#include "numerical/estimators/offline/PolynomialFitting.hpp"
#include "numerical/math/Tolerance.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    class TestPolynomialFitting : public ::testing::Test
    {
    protected:
        estimators::PolynomialFitting<float, 8, 2> fitter;
    };
}

TEST_F(TestPolynomialFitting, recovers_exact_line)
{
    estimators::PolynomialFitting<float, 8, 1> linFitter;
    math::Matrix<float, 8, 1> x;
    math::Matrix<float, 8, 1> y;

    for (std::size_t i = 0; i < 8; ++i)
    {
        float xi = static_cast<float>(i);
        x.at(i, 0) = xi;
        y.at(i, 0) = 2.0f + 3.0f * xi;
    }

    linFitter.Fit(x, y);
    const auto& c = linFitter.Coefficients();

    EXPECT_NEAR(c.at(0, 0), 2.0f, math::Tolerance<float>());
    EXPECT_NEAR(c.at(1, 0), 3.0f, math::Tolerance<float>());
}

TEST_F(TestPolynomialFitting, recovers_exact_quadratic)
{
    math::Matrix<float, 8, 1> x;
    math::Matrix<float, 8, 1> y;

    for (std::size_t i = 0; i < 8; ++i)
    {
        float xi = -1.0f + static_cast<float>(i) * (2.0f / 7.0f);
        x.at(i, 0) = xi;
        y.at(i, 0) = 1.0f - 0.5f * xi + 0.25f * xi * xi;
    }

    fitter.Fit(x, y);

    for (std::size_t i = 0; i < 8; ++i)
    {
        float xi = x.at(i, 0);
        float expected = 1.0f - 0.5f * xi + 0.25f * xi * xi;
        EXPECT_NEAR(fitter.Predict(xi), expected, math::Tolerance<float>());
    }
}

TEST_F(TestPolynomialFitting, fits_noisy_data_least_squares)
{
    math::Matrix<float, 8, 1> x;
    math::Matrix<float, 8, 1> y;

    static constexpr float noise[] = { 0.01f, -0.01f, 0.005f, -0.005f, 0.008f, -0.008f, 0.003f, -0.003f };

    for (std::size_t i = 0; i < 8; ++i)
    {
        float xi = static_cast<float>(i) * 0.25f;
        x.at(i, 0) = xi;
        y.at(i, 0) = 1.0f - 0.5f * xi + 0.25f * xi * xi + noise[i];
    }

    fitter.Fit(x, y);
    const auto& c = fitter.Coefficients();

    EXPECT_NEAR(c.at(0, 0), 1.0f, 0.05f);
    EXPECT_NEAR(c.at(1, 0), -0.5f, 0.05f);
    EXPECT_NEAR(c.at(2, 0), 0.25f, 0.05f);
}

TEST_F(TestPolynomialFitting, predict_uses_horner)
{
    math::Matrix<float, 8, 1> x;
    math::Matrix<float, 8, 1> y;

    for (std::size_t i = 0; i < 8; ++i)
    {
        float xi = static_cast<float>(i) * 0.5f;
        x.at(i, 0) = xi;
        y.at(i, 0) = 1.0f + 2.0f * xi + 3.0f * xi * xi;
    }

    fitter.Fit(x, y);
    const auto& c = fitter.Coefficients();

    float xVal = 1.5f;
    float horner = c.at(2, 0) * xVal * xVal + c.at(1, 0) * xVal + c.at(0, 0);

    EXPECT_NEAR(fitter.Predict(xVal), horner, math::Tolerance<float>());
}

TEST_F(TestPolynomialFitting, constant_data_gives_constant_term)
{
    math::Matrix<float, 8, 1> x;
    math::Matrix<float, 8, 1> y;

    for (std::size_t i = 0; i < 8; ++i)
    {
        x.at(i, 0) = static_cast<float>(i);
        y.at(i, 0) = 7.0f;
    }

    fitter.Fit(x, y);
    const auto& c = fitter.Coefficients();

    EXPECT_NEAR(c.at(0, 0), 7.0f, math::Tolerance<float>());
    EXPECT_NEAR(c.at(1, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(c.at(2, 0), 0.0f, math::Tolerance<float>());
}

TEST_F(TestPolynomialFitting, centering_improves_conditioning)
{
    math::Matrix<float, 8, 1> x;
    math::Matrix<float, 8, 1> y;

    float xMean = 104.0f;
    for (std::size_t i = 0; i < 8; ++i)
    {
        float xi = 100.0f + static_cast<float>(i);
        float xc = xi - xMean;
        x.at(i, 0) = xc;
        y.at(i, 0) = 1.0f - 0.5f * xc + 0.25f * xc * xc;
    }

    fitter.Fit(x, y);
    const auto& c = fitter.Coefficients();

    EXPECT_TRUE(std::isfinite(c.at(0, 0)));
    EXPECT_TRUE(std::isfinite(c.at(1, 0)));
    EXPECT_TRUE(std::isfinite(c.at(2, 0)));
    EXPECT_NEAR(c.at(1, 0), -0.5f, math::Tolerance<float>());
    EXPECT_NEAR(c.at(2, 0), 0.25f, math::Tolerance<float>());
}

TEST_F(TestPolynomialFitting, degree_zero_is_mean)
{
    estimators::PolynomialFitting<float, 8, 0> constFitter;
    math::Matrix<float, 8, 1> x;
    math::Matrix<float, 8, 1> y;

    float sum = 0.0f;
    for (std::size_t i = 0; i < 8; ++i)
    {
        x.at(i, 0) = static_cast<float>(i);
        float yi = static_cast<float>(i) * 1.5f + 2.0f;
        y.at(i, 0) = yi;
        sum += yi;
    }

    constFitter.Fit(x, y);
    const auto& c = constFitter.Coefficients();

    EXPECT_NEAR(c.at(0, 0), sum / 8.0f, math::Tolerance<float>());
}
