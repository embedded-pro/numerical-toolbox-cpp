#include "toolbox/analysis/FastFourierTransformRadix2Impl.hpp"
#include "toolbox/controllers/test_doubles/Tolerance.hpp"
#include "toolbox/math/QNumber.hpp"
#include "gtest/gtest.h"
#include <array>
#include <cmath>

namespace
{
    template<typename T>
    T CalculateMagnitude(const math::Complex<T>& value)
    {
        auto real = math::ToFloat(value.Real());
        auto imag = math::ToFloat(value.Imaginary());
        return std::sqrt(real * real + imag * imag);
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

        MockTwiddleFactors<T> twiddleFactors{};
        std::optional<analysis::FastFourierTransformRadix2Impl<T, Length>> fft;
        typename VectorReal::template WithMaxSize<Length> timeDomain;
        typename VectorComplex::template WithMaxSize<Length> frequencyDomain;
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestFastFourierTransform, TestedTypes);
}

TYPED_TEST(TestFastFourierTransform, zero_input_produces_zero_output)
{
    this->fft.emplace(this->twiddleFactors);
    this->timeDomain.clear();
    this->timeDomain.resize(TestFastFourierTransform<TypeParam>::Length);

    auto& result = this->fft->Forward(this->timeDomain);

    for (const auto& value : result)
        EXPECT_NEAR(math::ToFloat(CalculateMagnitude(value)), 0.0f, controllers::GetTolerance<TypeParam>());
}

TYPED_TEST(TestFastFourierTransform, dc_signal_appears_in_zero_frequency_bin)
{
    this->fft.emplace(this->twiddleFactors);
    this->timeDomain.clear();
    this->timeDomain.resize(TestFastFourierTransform<TypeParam>::Length);
    std::fill(this->timeDomain.begin(), this->timeDomain.end(), TypeParam(0.1f));

    auto& result = this->fft->Forward(this->timeDomain);

    EXPECT_NEAR(math::ToFloat(CalculateMagnitude(result[0])),
        static_cast<float>(TestFastFourierTransform<TypeParam>::Length) * 0.1f,
        controllers::GetTolerance<TypeParam>());

    for (size_t i = 1; i < result.size(); ++i)
        EXPECT_NEAR(math::ToFloat(CalculateMagnitude(result[i])), 0.0f, controllers::GetTolerance<TypeParam>());
}

TYPED_TEST(TestFastFourierTransform, forward_and_inverse_transform_recovers_original_signal)
{
    this->fft.emplace(this->twiddleFactors);
    constexpr std::array<float, TestFastFourierTransform<TypeParam>::Length> signal = {
        0.1f, 0.07f, 0.0f, -0.07f, -0.1f, -0.07f, 0.0f, 0.07f
    };

    this->timeDomain.clear();
    for (const auto& value : signal)
        this->timeDomain.push_back(TypeParam(value));

    auto& frequency = this->fft->Forward(this->timeDomain);
    auto& recovered = this->fft->Inverse(frequency);

    for (size_t i = 0; i < signal.size(); ++i)
        EXPECT_NEAR(math::ToFloat(recovered[i]), signal[i], controllers::GetTolerance<TypeParam>());
}

TYPED_TEST(TestFastFourierTransform, nyquist_frequency_detection)
{
    this->fft.emplace(this->twiddleFactors);
    this->timeDomain.clear();

    for (size_t i = 0; i < TestFastFourierTransform<TypeParam>::Length; ++i)
        this->timeDomain.push_back(TypeParam((i % 2) ? -0.1f : 0.1f));

    auto& result = this->fft->Forward(this->timeDomain);

    EXPECT_NEAR(math::ToFloat(CalculateMagnitude(result[TestFastFourierTransform<TypeParam>::Length / 2])),
        static_cast<float>(TestFastFourierTransform<TypeParam>::Length) * 0.1f,
        controllers::GetTolerance<TypeParam>());
}
