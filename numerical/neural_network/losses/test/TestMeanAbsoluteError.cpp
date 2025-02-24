#include "numerical/neural_network/losses/MeanAbsoluteError.hpp"
#include "gmock/gmock.h"

namespace
{
    template<typename QNumberType, std::size_t Size>
    class MockRegularization
        : public neural_network::Regularization<QNumberType, Size>
    {
    public:
        using Vector = typename neural_network::Regularization<QNumberType, Size>::Vector;
        MOCK_METHOD(QNumberType, Calculate, (const Vector& parameters), (const, override));
    };

    template<typename T>
    class TestMeanAbsoluteError
        : public ::testing::Test
    {
    public:
        static constexpr std::size_t NumberOfFeature = 2;
        using Vector = math::Matrix<T, NumberOfFeature, 1>;

        MockRegularization<T, NumberOfFeature> mockRegularization;
        Vector target;
        std::optional<neural_network::MeanAbsoluteError<T, NumberOfFeature>> loss;

        void SetUp() override
        {
            target = Vector{ T(0.5f), T(-0.3f) };
        }
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestMeanAbsoluteError, TestedTypes);
}

TYPED_TEST(TestMeanAbsoluteError, CostCallsRegularization)
{
    typename TestMeanAbsoluteError<TypeParam>::Vector parameters{ TypeParam(0.7f), TypeParam(-0.1f) };
    TypeParam regValue = TypeParam(0.1f);

    EXPECT_CALL(this->mockRegularization, Calculate(::testing::_))
        .WillOnce(::testing::Return(regValue));

    this->loss.emplace(this->target, this->mockRegularization);
    TypeParam cost = this->loss->Cost(parameters);

    TypeParam maeValue = TypeParam(0.0f);
    TypeParam diff1 = parameters[0] - this->target[0];
    TypeParam diff2 = parameters[1] - this->target[1];
    maeValue += (diff1 < TypeParam(0.0f) ? -diff1 : diff1) + (diff2 < TypeParam(0.0f) ? -diff2 : diff2);

    EXPECT_NEAR(math::ToFloat(cost), math::ToFloat(maeValue + regValue), 0.001f);
}

TYPED_TEST(TestMeanAbsoluteError, ZeroError)
{
    typename TestMeanAbsoluteError<TypeParam>::Vector parameters = this->target;
    TypeParam regValue = TypeParam(0.0f);

    EXPECT_CALL(this->mockRegularization, Calculate(::testing::_))
        .WillOnce(::testing::Return(regValue));

    this->loss.emplace(this->target, this->mockRegularization);
    TypeParam cost = this->loss->Cost(parameters);

    EXPECT_NEAR(math::ToFloat(cost), 0.0f, 0.001f);
}

TYPED_TEST(TestMeanAbsoluteError, MultipleParameters)
{
    std::array<typename TestMeanAbsoluteError<TypeParam>::Vector, 2> testParameters = {
        typename TestMeanAbsoluteError<TypeParam>::Vector{ TypeParam(0.4f), TypeParam(-0.4f) },
        typename TestMeanAbsoluteError<TypeParam>::Vector{ TypeParam(0.6f), TypeParam(-0.2f) }
    };

    TypeParam regValue = TypeParam(0.2f);

    EXPECT_CALL(this->mockRegularization, Calculate(::testing::_))
        .Times(testParameters.size())
        .WillRepeatedly(::testing::Return(regValue));

    this->loss.emplace(this->target, this->mockRegularization);

    for (const auto& params : testParameters)
    {
        TypeParam cost = this->loss->Cost(params);

        TypeParam maeValue = TypeParam(0.0f);
        TypeParam diff1 = params[0] - this->target[0];
        TypeParam diff2 = params[1] - this->target[1];
        maeValue += (diff1 < TypeParam(0.0f) ? -diff1 : diff1) + (diff2 < TypeParam(0.0f) ? -diff2 : diff2);

        EXPECT_NEAR(math::ToFloat(cost), math::ToFloat(maeValue + regValue), 0.001f);
    }
}

TYPED_TEST(TestMeanAbsoluteError, GradientTest)
{
    typename TestMeanAbsoluteError<TypeParam>::Vector parameters{ TypeParam(0.7f), TypeParam(-0.6f) };
    TypeParam regValue = TypeParam(0.1f);

    this->loss.emplace(this->target, this->mockRegularization);
    auto gradient = this->loss->Gradient(parameters);

    EXPECT_NEAR(math::ToFloat(gradient[0]), 0.9999f, 0.01f);
    EXPECT_NEAR(math::ToFloat(gradient[1]), -0.9999f, 0.01f);
}
