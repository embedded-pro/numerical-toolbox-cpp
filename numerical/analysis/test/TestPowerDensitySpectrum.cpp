#include "numerical/analysis/FastFourierTransform.hpp"
#include "numerical/analysis/PowerDensitySpectrum.hpp"
#include "numerical/math/Tolerance.hpp"
#include "numerical/windowing/Windowing.hpp"
#include "gmock/gmock.h"

namespace
{
    template<typename QNumberType>
    class WindowStub
        : public windowing::Window<QNumberType>
    {
    public:
        QNumberType operator()(std::size_t n, std::size_t order) override
        {
            return QNumberType(0.5f);
        }

        QNumberType Power(std::size_t order) override
        {
            return QNumberType(0.25f);
        }
    };

    template<typename QNumberType, std::size_t Length>
    class TwiddleFactorsStub
        : public analysis::TwiddleFactors<QNumberType, Length>
    {
    public:
        math::Complex<QNumberType>& operator[](std::size_t n) override
        {
            static math::Complex<QNumberType> factor(QNumberType(1.0f), QNumberType(0.0f));
            return factor;
        }
    };

    template<typename QNumberType, std::size_t Length>
    class FftStub
        : public analysis::FastFourierTransform<QNumberType>
    {
    public:
        using VectorComplex = typename analysis::FastFourierTransform<QNumberType>::VectorComplex;
        using VectorReal = typename analysis::FastFourierTransform<QNumberType>::VectorReal;

        explicit FftStub(analysis::TwiddleFactors<QNumberType, Length / 2>&)
        {}

        VectorComplex& Forward(VectorReal& input) override
        {
            result.clear();

            for (std::size_t i = 0; i < Length; ++i)
            {
                if (i == 0)
                    result.push_back(math::Complex<QNumberType>(QNumberType(0.5f), QNumberType(0.0f)));
                else
                    result.push_back(math::Complex<QNumberType>(QNumberType(0.0f), QNumberType(0.0f)));
            }
            return result;
        }

        VectorReal& Inverse(VectorComplex& input) override
        {
            timeResult.clear();
            for (std::size_t i = 0; i < Length; ++i)
                timeResult.push_back(QNumberType(0.0f));
            return timeResult;
        }

    private:
        typename VectorComplex::template WithMaxSize<Length> result;
        typename VectorReal::template WithMaxSize<Length> timeResult;
    };

    template<typename T>
    class TestPowerSpectralDensity
        : public ::testing::Test
    {
    public:
        static constexpr std::size_t length = 512;
        static constexpr std::size_t overlap = 50;

        using Fft = FftStub<T, length>;
        using Twiddle = TwiddleFactorsStub<T, length / 2>;
        using PowerDensitySpectrum = analysis::PowerSpectralDensity<T, length, Fft, Twiddle, overlap>;

        WindowStub<T> window;
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
    float tolerance = controllers::GetTolerance<TypeParam>();

    typename TestFixture::PowerDensitySpectrum::VectorReal::template WithMaxSize<2 * TestFixture::length> input;

    for (std::size_t i = 0; i < this->length * 2; ++i)
        input.push_back((i % 2) ? TypeParam(0.5f) : TypeParam(-0.5f));

    auto& spectrum1 = this->powerDensitySpectrum->Calculate(input);
    auto& spectrum2 = this->powerDensitySpectrum->Calculate(input);

    for (std::size_t i = 0; i < spectrum1.size(); ++i)
        EXPECT_NEAR(math::ToFloat(spectrum1[i]), math::ToFloat(spectrum2[i]), tolerance);
}
