#include "numerical/analysis/ConvolutionCorrelation.hpp"
#include "numerical/analysis/FastFourierTransformRadix2Impl.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gmock/gmock.h"
#include <array>
#include <cmath>
#include <numbers>

namespace
{
    template<typename T, std::size_t HalfLen>
    class ConcreteTwiddleFactors : public analysis::TwiddleFactors<T, HalfLen>
    {
    public:
        ConcreteTwiddleFactors()
        {
            for (std::size_t k = 0; k < HalfLen; ++k)
            {
                float angle{ -2.0f * std::numbers::pi_v<float> * static_cast<float>(k) / static_cast<float>(2 * HalfLen) };
                factors[k] = math::Complex<T>{ T(std::cos(angle)), T(std::sin(angle)) };
            }
        }

        math::Complex<T>& operator[](std::size_t n) override
        {
            return factors[n];
        }

    private:
        std::array<math::Complex<T>, HalfLen> factors;
    };

    class TestConvolutionCorrelation : public ::testing::Test
    {
    public:
        using Vec3 = infra::BoundedVector<float>::WithMaxSize<3>;
        using Vec4 = infra::BoundedVector<float>::WithMaxSize<4>;
        using Vec5 = infra::BoundedVector<float>::WithMaxSize<5>;
        using Vec7 = infra::BoundedVector<float>::WithMaxSize<7>;
        using Vec8 = infra::BoundedVector<float>::WithMaxSize<8>;
        using Vec9 = infra::BoundedVector<float>::WithMaxSize<9>;
    };
}

TEST_F(TestConvolutionCorrelation, linear_convolution_matches_hand_calc)
{
    Vec3 x;
    x.push_back(1.0f);
    x.push_back(2.0f);
    x.push_back(3.0f);

    Vec3 h;
    h.push_back(0.0f);
    h.push_back(1.0f);
    h.push_back(0.5f);

    Vec5 y;
    analysis::LinearConvolution<float, 3, 3>(x, h, y);

    EXPECT_NEAR(y[0], 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(y[1], 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(y[2], 2.5f, math::Tolerance<float>());
    EXPECT_NEAR(y[3], 4.0f, math::Tolerance<float>());
    EXPECT_NEAR(y[4], 1.5f, math::Tolerance<float>());
}

TEST_F(TestConvolutionCorrelation, convolution_with_unit_impulse_is_identity)
{
    Vec3 x;
    x.push_back(1.0f);
    x.push_back(2.0f);
    x.push_back(3.0f);

    Vec3 h;
    h.push_back(1.0f);
    h.push_back(0.0f);
    h.push_back(0.0f);

    Vec5 y;
    analysis::LinearConvolution<float, 3, 3>(x, h, y);

    EXPECT_NEAR(y[0], x[0], math::Tolerance<float>());
    EXPECT_NEAR(y[1], x[1], math::Tolerance<float>());
    EXPECT_NEAR(y[2], x[2], math::Tolerance<float>());
    EXPECT_NEAR(y[3], 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(y[4], 0.0f, math::Tolerance<float>());
}

TEST_F(TestConvolutionCorrelation, convolution_is_commutative)
{
    Vec3 x;
    x.push_back(1.0f);
    x.push_back(2.0f);
    x.push_back(3.0f);

    Vec3 h;
    h.push_back(0.0f);
    h.push_back(1.0f);
    h.push_back(0.5f);

    Vec5 y1;
    Vec5 y2;
    analysis::LinearConvolution<float, 3, 3>(x, h, y1);
    analysis::LinearConvolution<float, 3, 3>(h, x, y2);

    for (std::size_t i = 0; i < 5; ++i)
        EXPECT_NEAR(y1[i], y2[i], math::Tolerance<float>());
}

TEST_F(TestConvolutionCorrelation, linear_convolution_is_linear)
{
    Vec3 x1;
    x1.push_back(1.0f);
    x1.push_back(0.0f);
    x1.push_back(-1.0f);

    Vec3 x2;
    x2.push_back(0.0f);
    x2.push_back(1.0f);
    x2.push_back(2.0f);

    Vec3 h;
    h.push_back(1.0f);
    h.push_back(2.0f);
    h.push_back(1.0f);

    constexpr float a{ 3.0f };
    constexpr float b{ -2.0f };

    Vec3 xCombined;
    xCombined.push_back(a * x1[0] + b * x2[0]);
    xCombined.push_back(a * x1[1] + b * x2[1]);
    xCombined.push_back(a * x1[2] + b * x2[2]);

    Vec5 yCombined;
    analysis::LinearConvolution<float, 3, 3>(xCombined, h, yCombined);

    Vec5 y1;
    Vec5 y2;
    analysis::LinearConvolution<float, 3, 3>(x1, h, y1);
    analysis::LinearConvolution<float, 3, 3>(x2, h, y2);

    for (std::size_t i = 0; i < 5; ++i)
        EXPECT_NEAR(yCombined[i], a * y1[i] + b * y2[i], math::Tolerance<float>());
}

TEST_F(TestConvolutionCorrelation, circular_convolution_wraps)
{
    Vec4 x;
    x.push_back(1.0f);
    x.push_back(2.0f);
    x.push_back(3.0f);
    x.push_back(4.0f);

    Vec4 h;
    h.push_back(1.0f);
    h.push_back(0.0f);
    h.push_back(0.0f);
    h.push_back(1.0f);

    Vec4 y;
    analysis::CircularConvolution<float, 4>(x, h, y);

    EXPECT_NEAR(y[0], 3.0f, math::Tolerance<float>());
    EXPECT_NEAR(y[1], 5.0f, math::Tolerance<float>());
    EXPECT_NEAR(y[2], 7.0f, math::Tolerance<float>());
    EXPECT_NEAR(y[3], 5.0f, math::Tolerance<float>());
}

TEST_F(TestConvolutionCorrelation, autocorrelation_values_match_closed_form)
{
    Vec4 x;
    x.push_back(1.0f);
    x.push_back(1.0f);
    x.push_back(1.0f);
    x.push_back(1.0f);

    Vec7 r;
    analysis::AutoCorrelation<float, 4>(x, r);

    EXPECT_NEAR(r[0], 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(r[1], 2.0f, math::Tolerance<float>());
    EXPECT_NEAR(r[2], 3.0f, math::Tolerance<float>());
    EXPECT_NEAR(r[3], 4.0f, math::Tolerance<float>());
    EXPECT_NEAR(r[4], 3.0f, math::Tolerance<float>());
    EXPECT_NEAR(r[5], 2.0f, math::Tolerance<float>());
    EXPECT_NEAR(r[6], 1.0f, math::Tolerance<float>());
}

TEST_F(TestConvolutionCorrelation, autocorrelation_zero_lag_equals_signal_energy)
{
    Vec4 x;
    x.push_back(1.0f);
    x.push_back(2.0f);
    x.push_back(-1.0f);
    x.push_back(3.0f);

    Vec7 r;
    analysis::AutoCorrelation<float, 4>(x, r);

    float energy{ 0.0f };
    for (std::size_t i = 0; i < 4; ++i)
        energy += x[i] * x[i];

    EXPECT_NEAR(r[3], energy, math::Tolerance<float>());
}

TEST_F(TestConvolutionCorrelation, autocorrelation_is_symmetric)
{
    Vec4 x;
    x.push_back(1.0f);
    x.push_back(1.0f);
    x.push_back(1.0f);
    x.push_back(1.0f);

    Vec7 r;
    analysis::AutoCorrelation<float, 4>(x, r);

    std::size_t peak{ analysis::ArgMaxLag<float, 7>(r) };
    EXPECT_EQ(peak, 3u);

    EXPECT_NEAR(r[0], r[6], math::Tolerance<float>());
    EXPECT_NEAR(r[1], r[5], math::Tolerance<float>());
    EXPECT_NEAR(r[2], r[4], math::Tolerance<float>());
}

TEST_F(TestConvolutionCorrelation, cross_correlation_values_match_hand_calc)
{
    using Vec5 = infra::BoundedVector<float>::WithMaxSize<5>;
    using Vec9 = infra::BoundedVector<float>::WithMaxSize<9>;

    Vec5 x;
    x.push_back(1.0f);
    x.push_back(0.0f);
    x.push_back(0.0f);
    x.push_back(0.0f);
    x.push_back(0.0f);

    Vec5 yShifted;
    yShifted.push_back(0.0f);
    yShifted.push_back(0.0f);
    yShifted.push_back(0.0f);
    yShifted.push_back(1.0f);
    yShifted.push_back(0.0f);

    Vec9 r;
    analysis::CrossCorrelation<float, 5, 5>(x, yShifted, r);

    std::size_t lag{ analysis::ArgMaxLag<float, 9>(r) };
    EXPECT_EQ(lag, 1u);

    EXPECT_NEAR(r[1], 1.0f, math::Tolerance<float>());
    for (std::size_t i = 0; i < 9; ++i)
    {
        if (i != 1)
            EXPECT_NEAR(r[i], 0.0f, math::Tolerance<float>());
    }
}

TEST_F(TestConvolutionCorrelation, linear_convolution_zero_signal_gives_zero_output)
{
    Vec3 x;
    x.push_back(0.0f);
    x.push_back(0.0f);
    x.push_back(0.0f);

    Vec3 h;
    h.push_back(1.0f);
    h.push_back(2.0f);
    h.push_back(3.0f);

    Vec5 y;
    analysis::LinearConvolution<float, 3, 3>(x, h, y);

    for (std::size_t i = 0; i < 5; ++i)
        EXPECT_NEAR(y[i], 0.0f, math::Tolerance<float>());
}

TEST_F(TestConvolutionCorrelation, fast_convolution_matches_direct)
{
    Vec3 x;
    x.push_back(1.0f);
    x.push_back(2.0f);
    x.push_back(3.0f);

    Vec3 h;
    h.push_back(1.0f);
    h.push_back(1.0f);
    h.push_back(0.0f);

    Vec5 yDirect;
    analysis::LinearConvolution<float, 3, 3>(x, h, yDirect);

    ConcreteTwiddleFactors<float, 4> twiddle;
    analysis::FastFourierTransformRadix2Impl<float, 8> fft{ twiddle };

    Vec5 yFast;
    analysis::FastConvolution<float, 3, 3, 8>(x, h, yFast, fft);

    for (std::size_t i = 0; i < 5; ++i)
        EXPECT_NEAR(yFast[i], yDirect[i], 1e-3f);
}
