#include "numerical/neural_network/losses/MeanSquaredError.hpp"
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
    class TestMeanSquaredError
        : public ::testing::Test
    {
    public:
        static constexpr std::size_t NumberOfFeature = 2;
        using Vector = math::Matrix<T, NumberOfFeature, 1>;

        MockRegularization<T, NumberOfFeature> mockRegularization;
        Vector target;
        std::optional<neural_network::MeanSquaredError<T, NumberOfFeature>> loss;

        void SetUp() override
        {
            target = Vector{ T(0.5f), T(-0.3f) };
        }
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestMeanSquaredError, TestedTypes);
}

TYPED_TEST(TestMeanSquaredError, CostCallsRegularization)
{
    typename TestMeanSquaredError<TypeParam>::Vector parameters{ TypeParam(0.7f), TypeParam(-0.1f) };
    TypeParam regValue = TypeParam(0.1f);

    EXPECT_CALL(this->mockRegularization, Calculate(::testing::_))
        .WillOnce(::testing::Return(regValue));

    this->loss.emplace(this->target, this->mockRegularization);
    TypeParam cost = this->loss->Cost(parameters);

    TypeParam mseValue = TypeParam(0.0f);
    TypeParam diff1 = parameters[0] - this->target[0];
    TypeParam diff2 = parameters[1] - this->target[1];
    mseValue += diff1 * diff1 + diff2 * diff2;
    mseValue = TypeParam(math::ToFloat(mseValue) / 2.0f);

    EXPECT_NEAR(math::ToFloat(cost), math::ToFloat(mseValue + regValue), 0.001f);
}

TYPED_TEST(TestMeanSquaredError, ZeroError)
{
    typename TestMeanSquaredError<TypeParam>::Vector parameters = this->target;
    TypeParam regValue = TypeParam(0.0f);

    EXPECT_CALL(this->mockRegularization, Calculate(::testing::_))
        .WillOnce(::testing::Return(regValue));

    this->loss.emplace(this->target, this->mockRegularization);
    TypeParam cost = this->loss->Cost(parameters);

    EXPECT_NEAR(math::ToFloat(cost), 0.0f, 0.001f);
}

TYPED_TEST(TestMeanSquaredError, MultipleParameters)
{
    std::array<typename TestMeanSquaredError<TypeParam>::Vector, 2> testParameters = {
        typename TestMeanSquaredError<TypeParam>::Vector{ TypeParam(0.4f), TypeParam(-0.4f) },
        typename TestMeanSquaredError<TypeParam>::Vector{ TypeParam(0.6f), TypeParam(-0.2f) }
    };

    TypeParam regValue = TypeParam(0.2f);

    EXPECT_CALL(this->mockRegularization, Calculate(::testing::_))
        .Times(testParameters.size())
        .WillRepeatedly(::testing::Return(regValue));

    this->loss.emplace(this->target, this->mockRegularization);

    for (const auto& params : testParameters)
    {
        TypeParam cost = this->loss->Cost(params);

        TypeParam mseValue = TypeParam(0.0f);
        TypeParam diff1 = params[0] - this->target[0];
        TypeParam diff2 = params[1] - this->target[1];
        mseValue += diff1 * diff1 + diff2 * diff2;
        mseValue = TypeParam(math::ToFloat(mseValue) / 2.0f);

        EXPECT_NEAR(math::ToFloat(cost), math::ToFloat(mseValue + regValue), 0.001f);
    }
}
