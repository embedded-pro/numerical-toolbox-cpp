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

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestPowerSpectralDensity, TestedTypes);
}

TYPED_TEST(TestPowerSpectralDensity, when_input_smaller_than_fft_size_throws_assertion)
{
    typename TestFixture::PowerDensitySpectrum::VectorReal::template WithMaxSize<TestFixture::length> input;
    for (std::size_t i = 0; i < this->length - 1; ++i)
        input.push_back(TypeParam(0.1f));

    EXPECT_DEATH(this->powerDensitySpectrum->Calculate(input), "");
}

TYPED_TEST(TestPowerSpectralDensity, overlapping_segments_are_properly_averaged)
{
    float tolerance = math::Tolerance<TypeParam>();

    typename TestFixture::PowerDensitySpectrum::VectorReal::template WithMaxSize<2 * TestFixture::length> input;

    for (std::size_t i = 0; i < this->length * 2; ++i)
        input.push_back((i % 2) ? TypeParam(0.5f) : TypeParam(-0.5f));

    auto& spectrum1 = this->powerDensitySpectrum->Calculate(input);
    auto& spectrum2 = this->powerDensitySpectrum->Calculate(input);

    for (std::size_t i = 0; i < spectrum1.size(); ++i)
        EXPECT_NEAR(math::ToFloat(spectrum1[i]), math::ToFloat(spectrum2[i]), tolerance);
}
