#include "numerical/analysis/FastFourierTransformRadix2Impl.hpp"
#include "numerical/analysis/RealFastFourierTransform.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gmock/gmock.h"
#include <array>
#include <cmath>
#include <numbers>

namespace
{
    class TwiddleFactors8 : public analysis::TwiddleFactors<float, 4>
    {
    public:
        TwiddleFactors8()
        {
            for (std::size_t k{ 0 }; k < 4; ++k)
            {
                float angle{ -2.0f * std::numbers::pi_v<float> * static_cast<float>(k) / 8.0f };
                factors[k] = math::Complex<float>{ std::cos(angle), std::sin(angle) };
            }
        }

        math::Complex<float>& operator[](std::size_t n) override
        {
            return factors[n];
        }

    private:
        std::array<math::Complex<float>, 4> factors{};
    };

    class TwiddleFactors16 : public analysis::TwiddleFactors<float, 8>
    {
    public:
        TwiddleFactors16()
        {
            for (std::size_t k{ 0 }; k < 8; ++k)
            {
                float angle{ -2.0f * std::numbers::pi_v<float> * static_cast<float>(k) / 16.0f };
                factors[k] = math::Complex<float>{ std::cos(angle), std::sin(angle) };
            }
        }

        math::Complex<float>& operator[](std::size_t n) override
        {
            return factors[n];
        }

    private:
        std::array<math::Complex<float>, 8> factors{};
    };

    class TestRealFastFourierTransform : public ::testing::Test
    {
    public:
        static constexpr std::size_t N{ 16 };

        TwiddleFactors8 twiddleFactors{};
        TwiddleFactors16 realTwiddleFactors{};
        analysis::FastFourierTransformRadix2Impl<float, N / 2> engine{ twiddleFactors };
        analysis::RealFastFourierTransform<float, N> rfft{ engine, realTwiddleFactors };
        typename infra::BoundedVector<float>::template WithMaxSize<N> input{};
    };
}

TEST_F(TestRealFastFourierTransform, dc_input_has_only_dc_bin)
{
    input.resize(N, 1.0f);

    auto& spectrum{ rfft.Forward(input) };

    EXPECT_NEAR(spectrum[0].Real(), static_cast<float>(N), 1e-2f);
    for (std::size_t k{ 1 }; k <= N / 2; ++k)
    {
        float mag{ std::sqrt(spectrum[k].Real() * spectrum[k].Real() + spectrum[k].Imaginary() * spectrum[k].Imaginary()) };
        EXPECT_NEAR(mag, 0.0f, 1e-2f);
    }
}

TEST_F(TestRealFastFourierTransform, single_real_sinusoid_hits_one_bin)
{
    input.resize(N);
    for (std::size_t n{ 0 }; n < N; ++n)
        input[n] = std::cos(2.0f * std::numbers::pi_v<float> * 3.0f * static_cast<float>(n) / static_cast<float>(N));

    auto& spectrum{ rfft.Forward(input) };

    float mag3{ std::sqrt(spectrum[3].Real() * spectrum[3].Real() + spectrum[3].Imaginary() * spectrum[3].Imaginary()) };
    for (std::size_t k{ 0 }; k <= N / 2; ++k)
    {
        if (k == 3)
            continue;
        float mag{ std::sqrt(spectrum[k].Real() * spectrum[k].Real() + spectrum[k].Imaginary() * spectrum[k].Imaginary()) };
        EXPECT_LT(mag, mag3);
    }
}

TEST_F(TestRealFastFourierTransform, matches_reference_complex_fft)
{
    static constexpr std::array<float, N> signal{ 0.1f, 0.3f, -0.2f, 0.5f, 0.4f, -0.1f, 0.0f, 0.2f,
        -0.3f, 0.6f, 0.1f, -0.4f, 0.2f, 0.0f, -0.1f, 0.3f };
    input.resize(N);
    for (std::size_t n{ 0 }; n < N; ++n)
        input[n] = signal[n];

    auto& spectrum{ rfft.Forward(input) };

    for (std::size_t k{ 0 }; k <= N / 2; ++k)
    {
        float refReal{ 0.0f };
        float refImag{ 0.0f };
        for (std::size_t n{ 0 }; n < N; ++n)
        {
            float angle{ -2.0f * std::numbers::pi_v<float> * static_cast<float>(k) * static_cast<float>(n) / static_cast<float>(N) };
            refReal += signal[n] * std::cos(angle);
            refImag += signal[n] * std::sin(angle);
        }
        EXPECT_NEAR(spectrum[k].Real(), refReal, 1e-2f);
        EXPECT_NEAR(spectrum[k].Imaginary(), refImag, 1e-2f);
    }
}

TEST_F(TestRealFastFourierTransform, nyquist_and_dc_are_real)
{
    input.resize(N);
    for (std::size_t n{ 0 }; n < N; ++n)
        input[n] = std::cos(2.0f * std::numbers::pi_v<float> * static_cast<float>(n) / static_cast<float>(N));

    auto& spectrum{ rfft.Forward(input) };

    EXPECT_NEAR(spectrum[0].Imaginary(), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(spectrum[N / 2].Imaginary(), 0.0f, math::Tolerance<float>());
}

TEST_F(TestRealFastFourierTransform, inverse_is_left_inverse)
{
    static constexpr std::array<float, N> signal{ 0.5f, -0.3f, 0.1f, 0.8f, -0.6f, 0.2f, 0.0f, -0.4f,
        0.7f, -0.1f, 0.3f, -0.5f, 0.4f, 0.1f, -0.2f, 0.6f };
    input.resize(N);
    for (std::size_t n{ 0 }; n < N; ++n)
        input[n] = signal[n];

    auto& spectrum{ rfft.Forward(input) };
    auto& recovered{ rfft.Inverse(spectrum) };

    for (std::size_t n{ 0 }; n < N; ++n)
        EXPECT_NEAR(recovered[n], signal[n], 1e-2f);
}

TEST_F(TestRealFastFourierTransform, linearity_holds)
{
    static constexpr float a{ 2.0f };
    static constexpr float b{ 0.5f };
    static constexpr std::array<float, N> x1{ 1.0f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f,
        1.0f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f };
    static constexpr std::array<float, N> x2{ 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f,
        0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f };

    typename infra::BoundedVector<float>::template WithMaxSize<N> in1{};
    typename infra::BoundedVector<float>::template WithMaxSize<N> in2{};
    typename infra::BoundedVector<float>::template WithMaxSize<N> inCombined{};
    in1.resize(N);
    in2.resize(N);
    inCombined.resize(N);

    for (std::size_t n{ 0 }; n < N; ++n)
    {
        in1[n] = x1[n];
        in2[n] = x2[n];
        inCombined[n] = a * x1[n] + b * x2[n];
    }

    auto& s1{ rfft.Forward(in1) };
    typename infra::BoundedVector<math::Complex<float>>::template WithMaxSize<N / 2 + 1> s1Copy{};
    s1Copy.resize(N / 2 + 1);
    for (std::size_t k{ 0 }; k <= N / 2; ++k)
        s1Copy[k] = s1[k];

    auto& s2{ rfft.Forward(in2) };
    typename infra::BoundedVector<math::Complex<float>>::template WithMaxSize<N / 2 + 1> s2Copy{};
    s2Copy.resize(N / 2 + 1);
    for (std::size_t k{ 0 }; k <= N / 2; ++k)
        s2Copy[k] = s2[k];

    auto& sCombined{ rfft.Forward(inCombined) };

    for (std::size_t k{ 0 }; k <= N / 2; ++k)
    {
        float expectedReal{ a * s1Copy[k].Real() + b * s2Copy[k].Real() };
        float expectedImag{ a * s1Copy[k].Imaginary() + b * s2Copy[k].Imaginary() };
        EXPECT_NEAR(sCombined[k].Real(), expectedReal, 1e-2f);
        EXPECT_NEAR(sCombined[k].Imaginary(), expectedImag, 1e-2f);
    }
}

TEST_F(TestRealFastFourierTransform, impulse_has_flat_magnitude)
{
    input.resize(N, 0.0f);
    input[0] = 1.0f;

    auto& spectrum{ rfft.Forward(input) };

    for (std::size_t k{ 0 }; k <= N / 2; ++k)
    {
        float mag{ std::sqrt(spectrum[k].Real() * spectrum[k].Real() + spectrum[k].Imaginary() * spectrum[k].Imaginary()) };
        EXPECT_NEAR(mag, 1.0f, 1e-2f);
    }
}
