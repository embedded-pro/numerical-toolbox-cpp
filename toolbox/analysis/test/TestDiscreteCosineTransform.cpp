#include "toolbox/analysis/DiscreteCosineTransform.hpp"
#include "toolbox/math/QNumber.hpp"
#include "gmock/gmock.h"

namespace
{
    template<typename T>
    class MockFft
        : public analysis::FastFourierTransform<T>
    {
    public:
        MOCK_METHOD(typename analysis::FastFourierTransform<T>::VectorComplex&, Forward, (typename analysis::FastFourierTransform<T>::VectorReal & input), (override));
        MOCK_METHOD(typename analysis::FastFourierTransform<T>::VectorReal&, Inverse, (typename analysis::FastFourierTransform<T>::VectorComplex & input), (override));
    };

    template<typename T>
    class TestDiscreteConsineTransform
        : public ::testing::Test
    {
    public:
        static constexpr std::size_t Length = 8;
        using VectorReal = typename analysis::FastFourierTransform<T>::VectorReal;
        using VectorComplex = typename analysis::FastFourierTransform<T>::VectorComplex;

        MockFft<T> mockFft;
        std::optional<analysis::DiscreteConsineTransform<T, Length>> dct;
        typename VectorReal::template WithMaxSize<Length> input;
        typename VectorReal::template WithMaxSize<Length> output;
        typename VectorComplex::template WithMaxSize<Length> fftOutput;
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestDiscreteConsineTransform, TestedTypes);
}

TYPED_TEST(TestDiscreteConsineTransform, forward_transform_calls_fft_with_preprocessed_data)
{
    this->dct.emplace(this->mockFft);
    this->input.clear();
    this->input.resize(TestDiscreteConsineTransform<TypeParam>::Length);

    for (size_t i = 0; i < this->input.size(); ++i)
        this->input[i] = TypeParam(0.1f);

    EXPECT_CALL(this->mockFft, Forward(::testing::_))
        .WillOnce(::testing::Invoke([this](typename analysis::FastFourierTransform<TypeParam>::VectorReal& input) -> typename analysis::FastFourierTransform<TypeParam>::VectorComplex&
            {
                for (size_t i = 0; i < input.size() / 2; ++i)
                    EXPECT_EQ(math::ToFloat(input[i]), math::ToFloat(this->input[2 * i]));

                this->fftOutput.clear();
                this->fftOutput.resize(this->fftOutput.max_size());
                return this->fftOutput;
            }));

    this->dct->Forward(this->input);
}

TYPED_TEST(TestDiscreteConsineTransform, inverse_transform_calls_fft_with_preprocessed_data)
{
    this->dct.emplace(this->mockFft);
    this->input.clear();
    this->input.resize(TestDiscreteConsineTransform<TypeParam>::Length);

    for (size_t i = 0; i < this->input.size(); ++i)
        this->input[i] = TypeParam(0.1f);

    EXPECT_CALL(this->mockFft, Inverse(::testing::_))
        .WillOnce(::testing::Invoke([this](typename analysis::FastFourierTransform<TypeParam>::VectorComplex& input) -> typename analysis::FastFourierTransform<TypeParam>::VectorReal&
            {
                EXPECT_LE(std::abs(math::ToFloat(input[0].Real())), 1.0f);

                this->output.clear();
                this->output.resize(this->output.max_size());
                return this->output;
            }));

    this->dct->Inverse(this->input);
}

TYPED_TEST(TestDiscreteConsineTransform, input_size_matches_fft_size)
{
    this->dct.emplace(this->mockFft);
    this->input.clear();
    this->input.resize(TestDiscreteConsineTransform<TypeParam>::Length);

    EXPECT_CALL(this->mockFft, Forward(::testing::_))
        .WillOnce(::testing::Invoke([this](typename analysis::FastFourierTransform<TypeParam>::VectorReal& input) -> typename analysis::FastFourierTransform<TypeParam>::VectorComplex&
            {
                EXPECT_EQ(input.size(), TestDiscreteConsineTransform<TypeParam>::Length);
                this->fftOutput.clear();
                this->fftOutput.resize(this->fftOutput.max_size());
                return this->fftOutput;
            }));

    this->dct->Forward(this->input);
}

TYPED_TEST(TestDiscreteConsineTransform, scaling_stays_within_fixed_point_range)
{
    this->dct.emplace(this->mockFft);
    this->input.clear();
    this->input.resize(TestDiscreteConsineTransform<TypeParam>::Length);

    std::fill(this->input.begin(), this->input.end(), TypeParam(0.9999f));

    EXPECT_CALL(this->mockFft, Forward(::testing::_))
        .WillOnce(::testing::Invoke([this](typename analysis::FastFourierTransform<TypeParam>::VectorReal& input) -> typename analysis::FastFourierTransform<TypeParam>::VectorComplex&
            {
                for (const auto& value : input)
                    EXPECT_LE(std::abs(math::ToFloat(value)), 1.0f);

                this->fftOutput.clear();
                this->fftOutput.resize(this->fftOutput.max_size());
                return this->fftOutput;
            }));

    this->dct->Forward(this->input);
}
