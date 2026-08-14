#include "numerical/analysis/FastFourierTransformRadix2Impl.hpp"
#include "numerical/math/QNumber.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gtest/gtest.h"
#include <array>
#include <cmath>
#include <numbers>

namespace
{
    template<typename T>
    T CalculateMagnitude(const math::Complex<T>& value)
    {
        auto real = math::ToFloat(value.Real());
        auto imag = math::ToFloat(value.Imaginary());
        return T(std::sqrt(real * real + imag * imag));
    }

    template<typename T>
    class MockTwiddleFactors
        : public analysis::TwiddleFactors<T, 4>
    {
    public:
        MockTwiddleFactors()
        {
            factors[0] = math::Complex<T>{ T(0.9999f), T(0.0f) };
            factors[1] = math::Complex<T>{ T(0.0f), T(-0.9999f) };
            factors[2] = math::Complex<T>{ T(-0.9999f), T(0.0f) };
            factors[3] = math::Complex<T>{ T(0.0f), T(0.9999f) };
        }

        math::Complex<T>& operator[](std::size_t n) override
        {
            return factors[n];
        }

    private:
        std::array<math::Complex<T>, 4> factors;
    };

    template<typename T>
    class TestFastFourierTransform
        : public ::testing::Test
    {
    public:
        static constexpr std::size_t Length = 8;
        using VectorComplex = typename analysis::FastFourierTransform<T>::VectorComplex;
        using VectorReal = typename analysis::FastFourierTransform<T>::VectorReal;

        void SetUp() override
        {
            fft.emplace(twiddleFactors);
        }

        MockTwiddleFactors<T> twiddleFactors{};
        std::optional<analysis::FastFourierTransformRadix2Impl<T, Length>> fft;
        typename VectorReal::template WithMaxSize<Length> timeDomain;
        typename VectorComplex::template WithMaxSize<Length> frequencyDomain;
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestFastFourierTransform, TestedTypes);

    class RealTwiddleFactors8 : public analysis::TwiddleFactors<float, 4>
    {
    public:
        RealTwiddleFactors8()
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

    class TestFastFourierTransformFloat : public ::testing::Test
    {
    public:
        static constexpr std::size_t Length = 8;
        using VectorComplex = analysis::FastFourierTransform<float>::VectorComplex;
        using VectorReal = analysis::FastFourierTransform<float>::VectorReal;

        void SetUp() override
        {
            fft.emplace(twiddleFactors);
        }

        RealTwiddleFactors8 twiddleFactors{};
        std::optional<analysis::FastFourierTransformRadix2Impl<float, Length>> fft;
        typename VectorReal::template WithMaxSize<Length> timeDomain;
        typename VectorComplex::template WithMaxSize<Length> frequencyDomain;
    };

    class MockTwiddleFactorsDegenerate
        : public analysis::TwiddleFactors<float, 0>
    {
    public:
        math::Complex<float>& operator[](std::size_t) override
        {
            return dummy;
        }

    private:
        math::Complex<float> dummy{};
    };

    class TestFastFourierTransformCoverage : public ::testing::Test
    {};

    class TestFastFourierTransformSingleSample : public ::testing::Test
    {
    protected:
        MockTwiddleFactorsDegenerate tw;
        analysis::FastFourierTransformRadix2Impl<float, 1> fft{ tw };
    };
}

TYPED_TEST(TestFastFourierTransform, log2_runtime_both_branches)
{
    EXPECT_EQ(analysis::FastFourierTransform<TypeParam>::Log2(1), 0u);
    EXPECT_EQ(analysis::FastFourierTransform<TypeParam>::Log2(8), 3u);
}

TYPED_TEST(TestFastFourierTransform, zero_input_produces_zero_output)
{
    this->timeDomain.clear();
    this->timeDomain.resize(TestFastFourierTransform<TypeParam>::Length);

    auto& result = this->fft->Forward(this->timeDomain);

    for (const auto& value : result)
        EXPECT_NEAR(math::ToFloat(CalculateMagnitude(value)), 0.0f, math::Tolerance<TypeParam>());
}

TYPED_TEST(TestFastFourierTransform, dc_signal_appears_in_zero_frequency_bin)
{
    this->timeDomain.clear();
    this->timeDomain.resize(TestFastFourierTransform<TypeParam>::Length);
    std::fill(this->timeDomain.begin(), this->timeDomain.end(), TypeParam(0.1f));

    auto& result = this->fft->Forward(this->timeDomain);

    EXPECT_NEAR(math::ToFloat(CalculateMagnitude(result[0])),
        static_cast<float>(TestFastFourierTransform<TypeParam>::Length) * 0.1f,
        math::Tolerance<TypeParam>());

    for (size_t i = 1; i < result.size(); ++i)
        EXPECT_NEAR(math::ToFloat(CalculateMagnitude(result[i])), 0.0f, math::Tolerance<TypeParam>());
}

TYPED_TEST(TestFastFourierTransform, forward_and_inverse_transform_recovers_original_signal)
{
    constexpr std::array<float, TestFastFourierTransform<TypeParam>::Length> signal = {
        0.1f, 0.07f, 0.0f, -0.07f, -0.1f, -0.07f, 0.0f, 0.07f
    };

    this->timeDomain.clear();
    for (const auto& value : signal)
        this->timeDomain.push_back(TypeParam(value));

    auto& frequency = this->fft->Forward(this->timeDomain);
    auto& recovered = this->fft->Inverse(frequency);

    for (size_t i = 0; i < signal.size(); ++i)
        EXPECT_NEAR(math::ToFloat(recovered[i]), signal[i], math::Tolerance<TypeParam>());
}

TYPED_TEST(TestFastFourierTransform, nyquist_frequency_detection)
{
    this->timeDomain.clear();

    for (size_t i = 0; i < TestFastFourierTransform<TypeParam>::Length; ++i)
        this->timeDomain.push_back(TypeParam((i % 2) ? -0.1f : 0.1f));

    auto& result = this->fft->Forward(this->timeDomain);

    EXPECT_NEAR(math::ToFloat(CalculateMagnitude(result[TestFastFourierTransform<TypeParam>::Length / 2])),
        static_cast<float>(TestFastFourierTransform<TypeParam>::Length) * 0.1f,
        math::Tolerance<TypeParam>());
}

TEST_F(TestFastFourierTransformFloat, sinusoid_bin1_magnitude_and_phase_match_dft)
{
    constexpr std::size_t k{ 1 };
    timeDomain.resize(Length);
    for (std::size_t n{ 0 }; n < Length; ++n)
        timeDomain[n] = std::cos(2.0f * std::numbers::pi_v<float> * static_cast<float>(k) * static_cast<float>(n) / static_cast<float>(Length));

    auto& result = fft->Forward(timeDomain);

    float refReal{ 0.0f };
    float refImag{ 0.0f };
    for (std::size_t n{ 0 }; n < Length; ++n)
    {
        float angle{ -2.0f * std::numbers::pi_v<float> * static_cast<float>(k) * static_cast<float>(n) / static_cast<float>(Length) };
        refReal += timeDomain[n] * std::cos(angle);
        refImag += timeDomain[n] * std::sin(angle);
    }

    EXPECT_NEAR(result[k].Real(), refReal, 1e-3f);
    EXPECT_NEAR(result[k].Imaginary(), refImag, 1e-3f);
}

TEST_F(TestFastFourierTransformFloat, impulse_produces_flat_magnitude_spectrum)
{
    timeDomain.resize(Length, 0.0f);
    timeDomain[0] = 1.0f;

    auto& result = fft->Forward(timeDomain);

    for (std::size_t k{ 0 }; k < Length; ++k)
    {
        float mag{ std::sqrt(result[k].Real() * result[k].Real() + result[k].Imaginary() * result[k].Imaginary()) };
        EXPECT_NEAR(mag, 1.0f, 1e-3f);
    }
}

TEST_F(TestFastFourierTransformFloat, parseval_energy_preserved)
{
    constexpr std::array<float, 8> signal{ 0.2f, -0.4f, 0.6f, 0.1f, -0.3f, 0.5f, -0.1f, 0.8f };
    timeDomain.resize(Length);
    for (std::size_t n{ 0 }; n < Length; ++n)
        timeDomain[n] = signal[n];

    float timePower{ 0.0f };
    for (float s : signal)
        timePower += s * s;

    auto& result = fft->Forward(timeDomain);

    float freqPower{ 0.0f };
    for (std::size_t k{ 0 }; k < Length; ++k)
        freqPower += result[k].Real() * result[k].Real() + result[k].Imaginary() * result[k].Imaginary();
    freqPower /= static_cast<float>(Length);

    EXPECT_NEAR(freqPower, timePower, 1e-3f);
}

TEST_F(TestFastFourierTransformFloat, linearity_forward_transform)
{
    constexpr float a{ 3.0f };
    constexpr float b{ -2.0f };
    constexpr std::array<float, 8> x1{ 1.0f, 0.5f, -0.5f, -1.0f, -0.5f, 0.5f, 1.0f, 0.5f };
    constexpr std::array<float, 8> x2{ 0.3f, -0.1f, 0.4f, -0.2f, 0.5f, -0.3f, 0.1f, -0.4f };

    typename VectorReal::template WithMaxSize<Length> in1{};
    typename VectorReal::template WithMaxSize<Length> in2{};
    typename VectorReal::template WithMaxSize<Length> inCombined{};
    in1.resize(Length);
    in2.resize(Length);
    inCombined.resize(Length);

    for (std::size_t n{ 0 }; n < Length; ++n)
    {
        in1[n] = x1[n];
        in2[n] = x2[n];
        inCombined[n] = a * x1[n] + b * x2[n];
    }

    auto& s1{ fft->Forward(in1) };
    typename VectorComplex::template WithMaxSize<Length> s1Copy{};
    s1Copy.resize(Length);
    for (std::size_t k{ 0 }; k < Length; ++k)
        s1Copy[k] = s1[k];

    auto& s2{ fft->Forward(in2) };
    typename VectorComplex::template WithMaxSize<Length> s2Copy{};
    s2Copy.resize(Length);
    for (std::size_t k{ 0 }; k < Length; ++k)
        s2Copy[k] = s2[k];

    auto& sCombined{ fft->Forward(inCombined) };

    for (std::size_t k{ 0 }; k < Length; ++k)
    {
        EXPECT_NEAR(sCombined[k].Real(), a * s1Copy[k].Real() + b * s2Copy[k].Real(), 1e-3f);
        EXPECT_NEAR(sCombined[k].Imaginary(), a * s1Copy[k].Imaginary() + b * s2Copy[k].Imaginary(), 1e-3f);
    }
}

TEST_F(TestFastFourierTransformFloat, inverse_of_known_spectrum_recovers_time_domain)
{
    constexpr std::array<float, 8> expected{ 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

    frequencyDomain.resize(Length);
    for (std::size_t k{ 0 }; k < Length; ++k)
        frequencyDomain[k] = math::Complex<float>{ 1.0f, 0.0f };

    auto& result = fft->Inverse(frequencyDomain);

    for (std::size_t n{ 0 }; n < Length; ++n)
        EXPECT_NEAR(result[n], expected[n], 1e-3f);
}

TEST_F(TestFastFourierTransformFloat, all_bins_match_direct_dft)
{
    constexpr std::array<float, 8> signal{ 0.1f, 0.3f, -0.2f, 0.5f, 0.4f, -0.1f, 0.0f, 0.2f };
    timeDomain.resize(Length);
    for (std::size_t n{ 0 }; n < Length; ++n)
        timeDomain[n] = signal[n];

    auto& result = fft->Forward(timeDomain);

    for (std::size_t k{ 0 }; k < Length; ++k)
    {
        float refReal{ 0.0f };
        float refImag{ 0.0f };
        for (std::size_t n{ 0 }; n < Length; ++n)
        {
            float angle{ -2.0f * std::numbers::pi_v<float> * static_cast<float>(k) * static_cast<float>(n) / static_cast<float>(Length) };
            refReal += signal[n] * std::cos(angle);
            refImag += signal[n] * std::sin(angle);
        }
        EXPECT_NEAR(result[k].Real(), refReal, 1e-3f);
        EXPECT_NEAR(result[k].Imaginary(), refImag, 1e-3f);
    }
}

TEST_F(TestFastFourierTransformCoverage, twiddle_factors_virtual_destructor)
{
    MockTwiddleFactors<float> tf;
    (void)tf;
}

TEST_F(TestFastFourierTransformCoverage, fft_base_virtual_destructor)
{
    MockTwiddleFactors<float> tw;
    analysis::FastFourierTransformRadix2Impl<float, 8> fft{ tw };
    (void)fft;
}

TEST_F(TestFastFourierTransformCoverage, log2_static_one_returns_zero)
{
    EXPECT_EQ(analysis::FastFourierTransform<float>::Log2(1), 0u);
}

TEST_F(TestFastFourierTransformCoverage, log2_static_eight_returns_three)
{
    EXPECT_EQ(analysis::FastFourierTransform<float>::Log2(8), 3u);
}

TEST_F(TestFastFourierTransformSingleSample, forward_single_sample_is_identity)
{
    typename analysis::FastFourierTransform<float>::VectorReal::template WithMaxSize<1> input;
    input.push_back(0.5f);

    auto& result = fft.Forward(input);

    EXPECT_NEAR(result[0].Real(), 0.5f, 1e-5f);
    EXPECT_NEAR(result[0].Imaginary(), 0.0f, 1e-5f);
}
