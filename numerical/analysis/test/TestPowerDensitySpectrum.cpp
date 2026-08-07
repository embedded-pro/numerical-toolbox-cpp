#include "numerical/analysis/PowerDensitySpectrum.hpp"
#include "numerical/analysis/test/PowerDensitySpectrumTestSupport.hpp"
#include "numerical/math/Tolerance.hpp"
#include "gmock/gmock.h"

namespace
{
    template<typename T>
    class TestPowerSpectralDensity
        : public ::testing::Test
    {
    public:
        static constexpr std::size_t length = 512;
        static constexpr std::size_t overlap = 50;

        using Fft = analysis::test::FftStub<T, length>;
        using Twiddle = analysis::test::TwiddleFactorsStub<T, length / 2>;
        using PowerDensitySpectrum = analysis::PowerSpectralDensity<T, length, Fft, Twiddle, overlap>;

        analysis::test::WindowStub<T> window;
        T samplingTime = T(1.0f / 48000.0f);
        std::optional<PowerDensitySpectrum> powerDensitySpectrum;

        void SetUp() override
        {
            powerDensitySpectrum.emplace(window, samplingTime);
        }
    };

    template<typename T>
    class TestPowerSpectralDensityZeroOverlap
        : public ::testing::Test
    {
    public:
        static constexpr std::size_t length = 512;
        static constexpr std::size_t overlap = 0;

        using Fft = analysis::test::FftStub<T, length>;
        using Twiddle = analysis::test::TwiddleFactorsStub<T, length / 2>;
        using PowerDensitySpectrum = analysis::PowerSpectralDensity<T, length, Fft, Twiddle, overlap>;

        analysis::test::WindowStub<T> window;
        T samplingTime = T(1.0f / 48000.0f);
        std::optional<PowerDensitySpectrum> powerDensitySpectrum;

        void SetUp() override
        {
            powerDensitySpectrum.emplace(window, samplingTime);
        }
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestPowerSpectralDensity, TestedTypes);
    TYPED_TEST_SUITE(TestPowerSpectralDensityZeroOverlap, TestedTypes);
}

TYPED_TEST(TestPowerSpectralDensity, when_input_smaller_than_fft_size_throws_assertion)
{
    typename TestFixture::PowerDensitySpectrum::VectorReal::template WithMaxSize<TestFixture::length> input;
    for (std::size_t i = 0; i < this->length - 1; ++i)
        input.push_back(TypeParam(0.1f));

    EXPECT_DEATH_IF_SUPPORTED(this->powerDensitySpectrum->Calculate(input), "");
}

TYPED_TEST(TestPowerSpectralDensity, output_size_equals_half_segment_size_plus_one)
{
    typename TestFixture::PowerDensitySpectrum::VectorReal::template WithMaxSize<TestFixture::length> input;
    for (std::size_t i = 0; i < this->length; ++i)
        input.push_back(TypeParam(0.5f));

    auto& result = this->powerDensitySpectrum->Calculate(input);

    EXPECT_EQ(result.size(), TestFixture::length / 2 + 1);
}

TYPED_TEST(TestPowerSpectralDensity, non_dc_bins_are_zero_for_stub_fft)
{
    float tolerance = math::Tolerance<TypeParam>();

    typename TestFixture::PowerDensitySpectrum::VectorReal::template WithMaxSize<TestFixture::length> input;
    for (std::size_t i = 0; i < this->length; ++i)
        input.push_back(TypeParam(0.5f));

    auto& result = this->powerDensitySpectrum->Calculate(input);

    for (std::size_t k = 1; k < result.size(); ++k)
        EXPECT_NEAR(math::ToFloat(result[k]), 0.0f, tolerance);
}

TYPED_TEST(TestPowerSpectralDensity, dc_bin_is_nonzero_for_stub_fft)
{
    if constexpr (!std::is_floating_point_v<TypeParam>)
        GTEST_SKIP();

    typename TestFixture::PowerDensitySpectrum::VectorReal::template WithMaxSize<TestFixture::length> input;
    for (std::size_t i = 0; i < this->length; ++i)
        input.push_back(TypeParam(0.5f));

    auto& result = this->powerDensitySpectrum->Calculate(input);

    EXPECT_GT(math::ToFloat(result[0]), 0.0f);
}

TYPED_TEST(TestPowerSpectralDensity, repeated_calculate_with_same_input_is_deterministic)
{
    float tolerance = math::Tolerance<TypeParam>();

    typename TestFixture::PowerDensitySpectrum::VectorReal::template WithMaxSize<TestFixture::length> input;
    for (std::size_t i = 0; i < this->length; ++i)
        input.push_back(TypeParam(0.5f));

    auto& result1 = this->powerDensitySpectrum->Calculate(input);
    std::array<float, TestFixture::length / 2 + 1> snapshot;
    for (std::size_t i = 0; i < result1.size(); ++i)
        snapshot[i] = math::ToFloat(result1[i]);

    auto& result2 = this->powerDensitySpectrum->Calculate(input);

    for (std::size_t i = 0; i < result2.size(); ++i)
        EXPECT_NEAR(math::ToFloat(result2[i]), snapshot[i], tolerance);
}

TYPED_TEST(TestPowerSpectralDensity, welch_averaging_produces_segment_count_independent_dc_bin)
{
    if constexpr (!std::is_floating_point_v<TypeParam>)
        GTEST_SKIP();

    typename TestFixture::PowerDensitySpectrum::VectorReal::template WithMaxSize<TestFixture::length> inputOne;
    for (std::size_t i = 0; i < this->length; ++i)
        inputOne.push_back(TypeParam(0.5f));

    typename TestFixture::PowerDensitySpectrum::VectorReal::template WithMaxSize<3 * TestFixture::length / 2> inputTwo;
    for (std::size_t i = 0; i < 3 * this->length / 2; ++i)
        inputTwo.push_back(TypeParam(0.5f));

    auto& r1 = this->powerDensitySpectrum->Calculate(inputOne);
    float dc1 = math::ToFloat(r1[0]);

    auto& r2 = this->powerDensitySpectrum->Calculate(inputTwo);
    float dc2 = math::ToFloat(r2[0]);

    ASSERT_GT(dc1, 0.0f);
    float ratio = dc2 / dc1;
    EXPECT_NEAR(ratio, 1.0f, 1e-3f);
}

TYPED_TEST(TestPowerSpectralDensity, dc_bin_equals_hand_computed_reference_for_single_segment)
{
    if constexpr (!std::is_floating_point_v<TypeParam>)
        GTEST_SKIP();

    typename TestFixture::PowerDensitySpectrum::VectorReal::template WithMaxSize<TestFixture::length> input;
    for (std::size_t i = 0; i < this->length; ++i)
        input.push_back(TypeParam(0.5f));

    auto& result = this->powerDensitySpectrum->Calculate(input);

    constexpr float samplingTimeF = 1.0f / 48000.0f;
    constexpr float segSizeInv = 1.0f / static_cast<float>(TestFixture::length);
    constexpr float windowPower = 0.25f;
    constexpr float magSq = 0.5f * 0.5f;
    constexpr float expected = (magSq * segSizeInv) * (samplingTimeF / windowPower);

    EXPECT_NEAR(math::ToFloat(result[0]), expected, expected * 1e-2f);
}

TYPED_TEST(TestPowerSpectralDensityZeroOverlap, output_size_equals_half_segment_size_plus_one)
{
    typename TestFixture::PowerDensitySpectrum::VectorReal::template WithMaxSize<2 * TestFixture::length> input;
    for (std::size_t i = 0; i < 2 * this->length; ++i)
        input.push_back(TypeParam(0.5f));

    auto& result = this->powerDensitySpectrum->Calculate(input);

    EXPECT_EQ(result.size(), TestFixture::length / 2 + 1);
}

TYPED_TEST(TestPowerSpectralDensityZeroOverlap, zero_overlap_step_equals_segment_size)
{
    if constexpr (!std::is_floating_point_v<TypeParam>)
        GTEST_SKIP();

    typename TestFixture::PowerDensitySpectrum::VectorReal::template WithMaxSize<TestFixture::length> inputOne;
    for (std::size_t i = 0; i < this->length; ++i)
        inputOne.push_back(TypeParam(0.5f));

    typename TestFixture::PowerDensitySpectrum::VectorReal::template WithMaxSize<2 * TestFixture::length> inputTwo;
    for (std::size_t i = 0; i < 2 * this->length; ++i)
        inputTwo.push_back(TypeParam(0.5f));

    auto& r1 = this->powerDensitySpectrum->Calculate(inputOne);
    float dc1 = math::ToFloat(r1[0]);

    auto& r2 = this->powerDensitySpectrum->Calculate(inputTwo);
    float dc2 = math::ToFloat(r2[0]);

    ASSERT_GT(dc1, 0.0f);
    float ratio = dc2 / dc1;
    EXPECT_NEAR(ratio, 1.0f, 1e-3f);
}
