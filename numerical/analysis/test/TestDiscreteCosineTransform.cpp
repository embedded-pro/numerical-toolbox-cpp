#include "numerical/analysis/DiscreteCosineTransform.hpp"
#include "numerical/analysis/FastFourierTransformRadix2Impl.hpp"
#include "numerical/math/QNumber.hpp"
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

    class MockFft : public analysis::FastFourierTransform<float>
    {
    public:
        MOCK_METHOD(VectorComplex&, Forward, (VectorReal & input), (override));
        MOCK_METHOD(VectorReal&, Inverse, (VectorComplex & input), (override));
    };

    class TestDiscreteCosineTransform : public ::testing::Test
    {
    public:
        static constexpr std::size_t Length = 8;

        using VectorReal = analysis::FastFourierTransform<float>::VectorReal;
        using VectorComplex = analysis::FastFourierTransform<float>::VectorComplex;
        using RealBuf = VectorReal::WithMaxSize<Length>;

        ConcreteTwiddleFactors<float, Length / 2> twiddle;
        analysis::FastFourierTransformRadix2Impl<float, Length> fft{ twiddle };
        analysis::DiscreteCosineTransform<float, Length> dct{ fft };

        RealBuf signal;

        void SetUp() override
        {
            signal.clear();
            signal.resize(Length);
        }
    };

    class TestDiscreteCosineTransformMockFft : public ::testing::Test
    {
    public:
        static constexpr std::size_t Length = 8;

        using VectorReal = analysis::FastFourierTransform<float>::VectorReal;
        using VectorComplex = analysis::FastFourierTransform<float>::VectorComplex;
        using RealBuf = VectorReal::WithMaxSize<Length>;
        using ComplexBuf = VectorComplex::WithMaxSize<Length>;

        ::testing::StrictMock<MockFft> mockFft;
        analysis::DiscreteCosineTransform<float, Length> dct{ mockFft };

        RealBuf signal;
        ComplexBuf fftOut;
        RealBuf ifftOut;

        void SetUp() override
        {
            signal.clear();
            signal.resize(Length);

            fftOut.clear();
            fftOut.resize(Length);

            ifftOut.clear();
            ifftOut.resize(Length);
        }
    };
}

TEST_F(TestDiscreteCosineTransform, forward_dc_signal_concentrates_energy_in_bin_zero)
{
    std::fill(signal.begin(), signal.end(), 1.0f);

    auto& result = dct.Forward(signal);

    EXPECT_NEAR(result[0], std::sqrt(static_cast<float>(Length)), math::Tolerance<float>());
    for (std::size_t k = 1; k < Length; ++k)
        EXPECT_NEAR(result[k], 0.0f, math::Tolerance<float>());
}

TEST_F(TestDiscreteCosineTransform, forward_impulse_at_origin_matches_closed_form)
{
    signal[0] = 1.0f;

    auto& result = dct.Forward(signal);

    constexpr float sqrtN{ 2.82842712f };
    EXPECT_NEAR(result[0], 1.0f / sqrtN, math::Tolerance<float>());
    for (std::size_t k = 1; k < Length; ++k)
    {
        float ref{ std::sqrt(2.0f / static_cast<float>(Length)) * std::cos(static_cast<float>(k) * std::numbers::pi_v<float> / (2.0f * static_cast<float>(Length))) };
        EXPECT_NEAR(result[k], ref, math::Tolerance<float>());
    }
}

TEST_F(TestDiscreteCosineTransform, forward_matches_direct_dct_ii_definition)
{
    constexpr std::array<float, Length> x{ 3.0f, -1.0f, 4.0f, 1.0f, -5.0f, 9.0f, -2.0f, 6.0f };
    for (std::size_t n = 0; n < Length; ++n)
        signal[n] = x[n];

    auto& result = dct.Forward(signal);

    float sum{ 0.0f };
    for (std::size_t n = 0; n < Length; ++n)
        sum += x[n];
    EXPECT_NEAR(result[0], sum / std::sqrt(static_cast<float>(Length)), math::Tolerance<float>());

    for (std::size_t k = 1; k < Length; ++k)
    {
        float acc{ 0.0f };
        for (std::size_t n = 0; n < Length; ++n)
            acc += x[n] * std::cos(std::numbers::pi_v<float> * (2.0f * static_cast<float>(n) + 1.0f) * static_cast<float>(k) / (2.0f * static_cast<float>(Length)));

        float ref{ std::sqrt(2.0f / static_cast<float>(Length)) * acc };
        EXPECT_NEAR(result[k], ref, math::Tolerance<float>());
    }
}

TEST_F(TestDiscreteCosineTransform, inverse_recovers_forward_input)
{
    constexpr std::array<float, Length> x{ 0.3f, -0.1f, 0.4f, 0.1f, -0.5f, 0.2f, -0.2f, 0.15f };
    for (std::size_t n = 0; n < Length; ++n)
        signal[n] = x[n];

    auto& coeffs = dct.Forward(signal);

    RealBuf spectrum;
    spectrum.resize(Length);
    for (std::size_t k = 0; k < Length; ++k)
        spectrum[k] = coeffs[k];

    auto& recovered = dct.Inverse(spectrum);

    for (std::size_t n = 0; n < Length; ++n)
        EXPECT_NEAR(recovered[n], x[n], math::Tolerance<float>());
}

TEST_F(TestDiscreteCosineTransform, forward_zero_input_gives_zero_output)
{
    auto& result = dct.Forward(signal);

    for (std::size_t k = 0; k < Length; ++k)
        EXPECT_NEAR(result[k], 0.0f, math::Tolerance<float>());
}

TEST_F(TestDiscreteCosineTransform, forward_is_linear)
{
    constexpr std::array<float, 8> x1{ 1.0f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f };
    constexpr std::array<float, 8> x2{ 0.0f, 1.0f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f, -1.0f };
    constexpr float a{ 0.5f };
    constexpr float b{ -0.25f };

    VectorReal::WithMaxSize<Length> buf1;
    VectorReal::WithMaxSize<Length> buf2;
    VectorReal::WithMaxSize<Length> bufComb;
    buf1.resize(Length);
    buf2.resize(Length);
    bufComb.resize(Length);

    for (std::size_t n = 0; n < Length; ++n)
    {
        buf1[n] = x1[n];
        buf2[n] = x2[n];
        bufComb[n] = a * x1[n] + b * x2[n];
    }

    auto& r1 = dct.Forward(buf1);
    std::array<float, 8> dct1{};
    for (std::size_t k = 0; k < Length; ++k)
        dct1[k] = r1[k];

    auto& r2 = dct.Forward(buf2);
    std::array<float, 8> dct2{};
    for (std::size_t k = 0; k < Length; ++k)
        dct2[k] = r2[k];

    auto& rComb = dct.Forward(bufComb);

    for (std::size_t k = 0; k < Length; ++k)
        EXPECT_NEAR(rComb[k], a * dct1[k] + b * dct2[k], math::Tolerance<float>());
}

TEST_F(TestDiscreteCosineTransform, inverse_zero_input_gives_zero_output)
{
    auto& result = dct.Inverse(signal);

    for (std::size_t n = 0; n < Length; ++n)
        EXPECT_NEAR(result[n], 0.0f, math::Tolerance<float>());
}

TEST_F(TestDiscreteCosineTransform, inverse_pure_dc_spectrum_gives_constant_signal)
{
    signal[0] = 1.0f;

    auto& result = dct.Inverse(signal);

    float ref{ 1.0f / std::sqrt(static_cast<float>(Length)) };
    for (std::size_t n = 0; n < Length; ++n)
        EXPECT_NEAR(result[n], ref, math::Tolerance<float>());
}

TEST_F(TestDiscreteCosineTransform, parseval_identity_holds)
{
    constexpr std::array<float, Length> x{ 3.0f, -1.0f, 4.0f, 1.0f, -5.0f, 9.0f, -2.0f, 6.0f };
    for (std::size_t n = 0; n < Length; ++n)
        signal[n] = x[n];

    auto& X = dct.Forward(signal);

    float energyTime{ 0.0f };
    for (std::size_t n = 0; n < Length; ++n)
        energyTime += x[n] * x[n];

    float energyFreq{ 0.0f };
    for (std::size_t k = 0; k < Length; ++k)
        energyFreq += X[k] * X[k];

    EXPECT_NEAR(energyFreq, energyTime, 1.0f);
}

TEST_F(TestDiscreteCosineTransformMockFft, forward_passes_full_length_buffer_to_fft)
{
    for (std::size_t i = 0; i < Length; ++i)
        signal[i] = 0.1f;

    EXPECT_CALL(mockFft, Forward(::testing::_))
        .WillOnce(::testing::Invoke([this](VectorReal& in) -> VectorComplex&
            {
                EXPECT_EQ(in.size(), Length);
                return fftOut;
            }));

    dct.Forward(signal);
}

TEST_F(TestDiscreteCosineTransformMockFft, inverse_builds_complex_buffer_within_valid_range)
{
    for (std::size_t i = 0; i < Length; ++i)
        signal[i] = 0.1f;

    EXPECT_CALL(mockFft, Inverse(::testing::_))
        .WillOnce(::testing::Invoke([this](VectorComplex& in) -> VectorReal&
            {
                EXPECT_EQ(in.size(), Length);
                for (std::size_t k = 0; k < in.size(); ++k)
                    EXPECT_LE(std::abs(math::ToFloat(in[k].Real())), 1.0f);
                return ifftOut;
            }));

    dct.Inverse(signal);
}

TEST_F(TestDiscreteCosineTransformMockFft, forward_scaling_stays_within_range_at_full_scale)
{
    std::fill(signal.begin(), signal.end(), 0.9999f);

    EXPECT_CALL(mockFft, Forward(::testing::_))
        .WillOnce(::testing::Invoke([this](VectorReal& in) -> VectorComplex&
            {
                for (const auto& v : in)
                    EXPECT_LE(std::abs(math::ToFloat(v)), 1.0f);
                return fftOut;
            }));

    dct.Forward(signal);
}
