#include "numerical/estimators/offline/TotalLeastSquares.hpp"
#include "numerical/math/Tolerance.hpp"
#include <array>
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    class TestTotalLeastSquares : public ::testing::Test
    {
    protected:
        estimators::TotalLeastSquares<float, 8, 1> tls;
        estimators::TotalLeastSquares<float, 10, 2> tls2;
    };
}

TEST_F(TestTotalLeastSquares, recovers_exact_line_no_noise)
{
    math::Matrix<float, 8, 1> a;
    math::Vector<float, 8> b;

    for (std::size_t i = 0; i < 8; ++i)
    {
        float ai = 1.0f + static_cast<float>(i);
        a.at(i, 0) = ai;
        b.at(i, 0) = 2.0f * ai;
    }

    ASSERT_TRUE(tls.Fit(a, b));
    EXPECT_NEAR(tls.Coefficients().at(0, 0), 2.0f, math::Tolerance<float>());
}

TEST_F(TestTotalLeastSquares, symmetric_noise_beats_ols)
{
    static constexpr std::array<float, 8> na = { 1.0f, -1.0f, -1.0f, 1.0f, 1.0f, -1.0f, -1.0f, 1.0f };
    static constexpr std::array<float, 8> nb = { 1.0f, -1.0f, 1.0f, -1.0f, -1.0f, 1.0f, -1.0f, 1.0f };

    math::Matrix<float, 8, 1> a;
    math::Vector<float, 8> b;

    float saa = 0.0f;
    float sab = 0.0f;
    for (std::size_t i = 0; i < 8; ++i)
    {
        float ai = (1.0f + static_cast<float>(i)) + na[i];
        float bi = 2.0f * (1.0f + static_cast<float>(i)) + nb[i];
        a.at(i, 0) = ai;
        b.at(i, 0) = bi;
        saa += ai * ai;
        sab += ai * bi;
    }

    ASSERT_TRUE(tls.Fit(a, b));

    float ols = sab / saa;
    float tlsSlope = tls.Coefficients().at(0, 0);

    EXPECT_LT(std::abs(tlsSlope - 2.0f), std::abs(ols - 2.0f));
    EXPECT_NEAR(tlsSlope, 2.0f, math::Tolerance<float>());
}

TEST_F(TestTotalLeastSquares, matches_ols_when_regressors_clean)
{
    static constexpr std::array<float, 8> nb = { 0.05f, -0.04f, 0.03f, -0.02f, 0.04f, -0.05f, 0.02f, -0.03f };

    math::Matrix<float, 8, 1> a;
    math::Vector<float, 8> b;

    float saa = 0.0f;
    float sab = 0.0f;
    for (std::size_t i = 0; i < 8; ++i)
    {
        float ai = 1.0f + static_cast<float>(i);
        float bi = 2.0f * ai + nb[i];
        a.at(i, 0) = ai;
        b.at(i, 0) = bi;
        saa += ai * ai;
        sab += ai * bi;
    }

    ASSERT_TRUE(tls.Fit(a, b));

    float ols = sab / saa;
    EXPECT_NEAR(tls.Coefficients().at(0, 0), ols, 1e-2f);
}

TEST_F(TestTotalLeastSquares, multivariate_plane_fit)
{
    math::Matrix<float, 10, 2> a;
    math::Vector<float, 10> b;

    for (std::size_t i = 0; i < 10; ++i)
    {
        float a1 = 1.0f + static_cast<float>(i);
        float a2 = 3.0f - 0.5f * static_cast<float>(i) + static_cast<float>(i % 3);
        a.at(i, 0) = a1;
        a.at(i, 1) = a2;
        b.at(i, 0) = 1.5f * a1 - 0.5f * a2;
    }

    ASSERT_TRUE(tls2.Fit(a, b));
    EXPECT_NEAR(tls2.Coefficients().at(0, 0), 1.5f, math::Tolerance<float>());
    EXPECT_NEAR(tls2.Coefficients().at(1, 0), -0.5f, math::Tolerance<float>());
}

TEST_F(TestTotalLeastSquares, degenerate_returns_false)
{
    math::Matrix<float, 8, 1> a;
    math::Vector<float, 8> b;

    for (std::size_t i = 0; i < 8; ++i)
    {
        a.at(i, 0) = 0.0f;
        b.at(i, 0) = 1.0f + static_cast<float>(i);
    }

    EXPECT_FALSE(tls.Fit(a, b));
}

TEST_F(TestTotalLeastSquares, predict_against_closed_form)
{
    math::Matrix<float, 10, 2> a;
    math::Vector<float, 10> b;

    for (std::size_t i = 0; i < 10; ++i)
    {
        float a1 = 1.0f + static_cast<float>(i);
        float a2 = 3.0f - 0.5f * static_cast<float>(i) + static_cast<float>(i % 3);
        a.at(i, 0) = a1;
        a.at(i, 1) = a2;
        b.at(i, 0) = 1.5f * a1 - 0.5f * a2;
    }

    ASSERT_TRUE(tls2.Fit(a, b));

    math::Vector<float, 2> x;
    x.at(0, 0) = 2.0f;
    x.at(1, 0) = -1.0f;

    float expected = 1.5f * 2.0f + (-0.5f) * (-1.0f);

    EXPECT_NEAR(tls2.Predict(x), expected, math::Tolerance<float>());
}

TEST_F(TestTotalLeastSquares, determinism_repeated_fit_yields_same_coefficients)
{
    math::Matrix<float, 8, 1> a;
    math::Vector<float, 8> b;

    for (std::size_t i = 0; i < 8; ++i)
    {
        float ai = 1.0f + static_cast<float>(i);
        a.at(i, 0) = ai;
        b.at(i, 0) = 3.0f * ai;
    }

    estimators::TotalLeastSquares<float, 8, 1> fresh;

    ASSERT_TRUE(tls.Fit(a, b));
    ASSERT_TRUE(fresh.Fit(a, b));

    EXPECT_FLOAT_EQ(tls.Coefficients().at(0, 0), fresh.Coefficients().at(0, 0));
}

TEST_F(TestTotalLeastSquares, coefficients_finite_after_failed_fit)
{
    math::Matrix<float, 8, 1> a;
    math::Vector<float, 8> b;

    for (std::size_t i = 0; i < 8; ++i)
    {
        a.at(i, 0) = 0.0f;
        b.at(i, 0) = 1.0f;
    }

    ASSERT_FALSE(tls.Fit(a, b));

    EXPECT_TRUE(std::isfinite(tls.Coefficients().at(0, 0)));
}

TEST_F(TestTotalLeastSquares, rmse_below_noise_bound_on_noisy_data)
{
    static constexpr std::array<float, 8> noise = { 0.05f, -0.03f, 0.04f, -0.05f, 0.02f, -0.04f, 0.03f, -0.02f };

    math::Matrix<float, 8, 1> a;
    math::Vector<float, 8> b;

    for (std::size_t i = 0; i < 8; ++i)
    {
        float ai = 1.0f + static_cast<float>(i);
        a.at(i, 0) = ai + noise[i];
        b.at(i, 0) = 3.0f * ai + noise[(i + 4) % 8];
    }

    ASSERT_TRUE(tls.Fit(a, b));

    float rmse = 0.0f;
    for (std::size_t i = 0; i < 8; ++i)
    {
        float ai = 1.0f + static_cast<float>(i);
        math::Vector<float, 1> x;
        x.at(0, 0) = a.at(i, 0);
        float err = tls.Predict(x) - b.at(i, 0);
        rmse += err * err;
    }
    rmse = std::sqrt(rmse / 8.0f);

    EXPECT_LT(rmse, 0.3f);
}

TEST_F(TestTotalLeastSquares, ill_conditioned_near_collinear_does_not_produce_nan)
{
    math::Matrix<float, 10, 2> a;
    math::Vector<float, 10> b;

    for (std::size_t i = 0; i < 10; ++i)
    {
        float ai = 1.0f + static_cast<float>(i);
        a.at(i, 0) = ai;
        a.at(i, 1) = ai + 1e-4f * static_cast<float>(i);
        b.at(i, 0) = 2.0f * ai;
    }

    bool ok = tls2.Fit(a, b);

    if (ok)
    {
        EXPECT_TRUE(std::isfinite(tls2.Coefficients().at(0, 0)));
        EXPECT_TRUE(std::isfinite(tls2.Coefficients().at(1, 0)));
    }
}
