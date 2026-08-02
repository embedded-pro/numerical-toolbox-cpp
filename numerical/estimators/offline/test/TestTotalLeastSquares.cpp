#include "numerical/estimators/offline/TotalLeastSquares.hpp"
#include "numerical/math/Tolerance.hpp"
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
    static constexpr float na[] = { 1.0f, -1.0f, -1.0f, 1.0f, 1.0f, -1.0f, -1.0f, 1.0f };
    static constexpr float nb[] = { 1.0f, -1.0f, 1.0f, -1.0f, -1.0f, 1.0f, -1.0f, 1.0f };

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
    static constexpr float nb[] = { 0.05f, -0.04f, 0.03f, -0.02f, 0.04f, -0.05f, 0.02f, -0.03f };

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

TEST_F(TestTotalLeastSquares, predict_matches_dot_product)
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

    const auto& c = tls2.Coefficients();
    float expected = c.at(0, 0) * x.at(0, 0) + c.at(1, 0) * x.at(1, 0);

    EXPECT_NEAR(tls2.Predict(x), expected, math::Tolerance<float>());
}
