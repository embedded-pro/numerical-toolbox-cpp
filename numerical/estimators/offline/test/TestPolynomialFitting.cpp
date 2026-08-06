#include "numerical/estimators/offline/PolynomialFitting.hpp"
#include "numerical/math/Tolerance.hpp"
#include <array>
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

TEST_F(TestPolynomialFitting, recovers_exact_line_coefficients)
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

TEST_F(TestPolynomialFitting, recovers_exact_quadratic_coefficients)
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
    const auto& c = fitter.Coefficients();

    EXPECT_NEAR(c.at(0, 0), 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(c.at(1, 0), -0.5f, math::Tolerance<float>());
    EXPECT_NEAR(c.at(2, 0), 0.25f, math::Tolerance<float>());
}

TEST_F(TestPolynomialFitting, predict_matches_doc_worked_example)
{
    estimators::PolynomialFitting<float, 4, 2> docFitter;
    math::Matrix<float, 4, 1> x;
    math::Matrix<float, 4, 1> y;

    x.at(0, 0) = 0.0f;
    y.at(0, 0) = 1.0f;
    x.at(1, 0) = 1.0f;
    y.at(1, 0) = 0.75f;
    x.at(2, 0) = 2.0f;
    y.at(2, 0) = 1.0f;
    x.at(3, 0) = 3.0f;
    y.at(3, 0) = 1.75f;

    docFitter.Fit(x, y);

    EXPECT_NEAR(docFitter.Predict(1.5f), 0.8125f, math::Tolerance<float>());
}

TEST_F(TestPolynomialFitting, fits_noisy_quadratic_coefficients_within_noise_bound)
{
    math::Matrix<float, 8, 1> x;
    math::Matrix<float, 8, 1> y;

    constexpr std::array<float, 8> noise = { 0.01f, -0.01f, 0.005f, -0.005f, 0.008f, -0.008f, 0.003f, -0.003f };

    for (std::size_t i = 0; i < 8; ++i)
    {
        float xi = static_cast<float>(i) * 0.25f;
        x.at(i, 0) = xi;
        y.at(i, 0) = 1.0f - 0.5f * xi + 0.25f * xi * xi + noise[i];
    }

    fitter.Fit(x, y);
    const auto& c = fitter.Coefficients();

    EXPECT_NEAR(c.at(0, 0), 1.0f, 0.02f);
    EXPECT_NEAR(c.at(1, 0), -0.5f, 0.02f);
    EXPECT_NEAR(c.at(2, 0), 0.25f, 0.02f);
}

TEST_F(TestPolynomialFitting, constant_data_gives_zero_higher_coefficients)
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

TEST_F(TestPolynomialFitting, centered_abscissae_recover_exact_quadratic_coefficients)
{
    math::Matrix<float, 8, 1> x;
    math::Matrix<float, 8, 1> y;

    float xMean = 104.0f;
    for (std::size_t i = 0; i < 8; ++i)
    {
        float xc = (100.0f + static_cast<float>(i)) - xMean;
        x.at(i, 0) = xc;
        y.at(i, 0) = 1.0f - 0.5f * xc + 0.25f * xc * xc;
    }

    fitter.Fit(x, y);
    const auto& c = fitter.Coefficients();

    EXPECT_NEAR(c.at(0, 0), 1.0f, math::Tolerance<float>());
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

TEST_F(TestPolynomialFitting, determinism_identical_fit_produces_identical_coefficients)
{
    math::Matrix<float, 8, 1> x;
    math::Matrix<float, 8, 1> y;

    for (std::size_t i = 0; i < 8; ++i)
    {
        float xi = static_cast<float>(i) * 0.5f;
        x.at(i, 0) = xi;
        y.at(i, 0) = 1.0f - 0.5f * xi + 0.25f * xi * xi;
    }

    estimators::PolynomialFitting<float, 8, 2> fitter2;
    fitter.Fit(x, y);
    fitter2.Fit(x, y);

    const auto& c1 = fitter.Coefficients();
    const auto& c2 = fitter2.Coefficients();

    EXPECT_FLOAT_EQ(c1.at(0, 0), c2.at(0, 0));
    EXPECT_FLOAT_EQ(c1.at(1, 0), c2.at(1, 0));
    EXPECT_FLOAT_EQ(c1.at(2, 0), c2.at(2, 0));
}

TEST_F(TestPolynomialFitting, predict_all_coefficients_finite_on_far_from_origin_data)
{
    math::Matrix<float, 8, 1> x;
    math::Matrix<float, 8, 1> y;

    for (std::size_t i = 0; i < 8; ++i)
    {
        float xi = static_cast<float>(i) * 0.5f;
        x.at(i, 0) = xi;
        y.at(i, 0) = 3.0f + 7.0f * xi + 2.0f * xi * xi;
    }

    fitter.Fit(x, y);
    const auto& c = fitter.Coefficients();

    EXPECT_TRUE(std::isfinite(c.at(0, 0)));
    EXPECT_TRUE(std::isfinite(c.at(1, 0)));
    EXPECT_TRUE(std::isfinite(c.at(2, 0)));
    EXPECT_NEAR(c.at(0, 0), 3.0f, math::Tolerance<float>());
    EXPECT_NEAR(c.at(1, 0), 7.0f, math::Tolerance<float>());
    EXPECT_NEAR(c.at(2, 0), 2.0f, math::Tolerance<float>());
}
