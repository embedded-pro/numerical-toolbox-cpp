#include "numerical/math/QNumber.hpp"
#include "numerical/neural_network/losses/CategoricalCrossEntropy.hpp"
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
    class TestCategoricalCrossEntropy
        : public ::testing::Test
    {
    public:
        static constexpr std::size_t NumberOfFeature = 2;
        using Vector = math::Matrix<T, NumberOfFeature, 1>;

        MockRegularization<T, NumberOfFeature> mockRegularization;
        Vector target;
        std::optional<neural_network::CategoricalCrossEntropy<T, NumberOfFeature>> loss;

        void SetUp() override
        {
            target = Vector{ T(0.8f), T(0.2f) };
        }
    };

    using TestedTypes = ::testing::Types<float>;
    TYPED_TEST_SUITE(TestCategoricalCrossEntropy, TestedTypes);
}

TYPED_TEST(TestCategoricalCrossEntropy, CostCallsRegularization)
{
    typename TestCategoricalCrossEntropy<TypeParam>::Vector parameters{ TypeParam(0.7f), TypeParam(0.3f) };
    TypeParam regValue = TypeParam(0.1f);

    EXPECT_CALL(this->mockRegularization, Calculate(::testing::_))
        .WillOnce(::testing::Return(regValue));

    this->loss.emplace(this->target, this->mockRegularization);
    TypeParam cost = this->loss->Cost(parameters);

    typename TestCategoricalCrossEntropy<TypeParam>::Vector probs;
    TypeParam sum = TypeParam(0.0f);

    for (std::size_t i = 0; i < this->NumberOfFeature; ++i)
    {
        probs[i] = std::exp(math::ToFloat(parameters[i]));
        sum += probs[i];
    }

    for (std::size_t i = 0; i < this->NumberOfFeature; ++i)
        probs[i] = probs[i] / sum;

    TypeParam expectedCost = TypeParam(0.0f);
    for (std::size_t i = 0; i < this->NumberOfFeature; ++i)
        expectedCost += -this->target[i] * std::log(math::ToFloat(probs[i]));

    EXPECT_NEAR(math::ToFloat(cost), math::ToFloat(expectedCost + regValue), 0.01f);
}

TYPED_TEST(TestCategoricalCrossEntropy, PerfectPrediction)
{
    typename TestCategoricalCrossEntropy<TypeParam>::Vector parameters{ TypeParam(2.0f), TypeParam(-2.0f) };
    TypeParam regValue = TypeParam(0.0f);

    EXPECT_CALL(this->mockRegularization, Calculate(::testing::_))
        .WillOnce(::testing::Return(regValue));

    this->loss.emplace(this->target, this->mockRegularization);
    TypeParam cost = this->loss->Cost(parameters);

    EXPECT_GT(math::ToFloat(cost), 0.0f);
    EXPECT_LT(math::ToFloat(cost), 1.0f);
}

TYPED_TEST(TestCategoricalCrossEntropy, GradientTest)
{
    typename TestCategoricalCrossEntropy<TypeParam>::Vector parameters{ TypeParam(0.2f), TypeParam(0.5f) };
    TypeParam regValue = TypeParam(0.1f);

    EXPECT_CALL(this->mockRegularization, Calculate(::testing::_))
        .WillOnce(::testing::Return(regValue));

    this->loss.emplace(this->target, this->mockRegularization);
    auto gradient = this->loss->Gradient(parameters);

    typename TestCategoricalCrossEntropy<TypeParam>::Vector probs;
    TypeParam sum = TypeParam(0.0f);

    for (std::size_t i = 0; i < this->NumberOfFeature; ++i)
    {
        probs[i] = std::exp(math::ToFloat(parameters[i]));
        sum += probs[i];
    }

    for (std::size_t i = 0; i < this->NumberOfFeature; ++i)
        probs[i] = probs[i] / sum;

    // Account for limited precision in Q types
    EXPECT_NEAR(math::ToFloat(gradient[0]), math::ToFloat(probs[0] - this->target[0] + regValue), 0.1f);
    EXPECT_NEAR(math::ToFloat(gradient[1]), math::ToFloat(probs[1] - this->target[1] + regValue), 0.1f);
}

TYPED_TEST(TestCategoricalCrossEntropy, MultipleParameters)
{
    std::array<typename TestCategoricalCrossEntropy<TypeParam>::Vector, 2> testParameters = {
        typename TestCategoricalCrossEntropy<TypeParam>::Vector{ TypeParam(0.3f), TypeParam(0.7f) },
        typename TestCategoricalCrossEntropy<TypeParam>::Vector{ TypeParam(0.1f), TypeParam(0.9f) }
    };

    TypeParam regValue = TypeParam(0.2f);

    EXPECT_CALL(this->mockRegularization, Calculate(::testing::_))
        .Times(testParameters.size())
        .WillRepeatedly(::testing::Return(regValue));

    this->loss.emplace(this->target, this->mockRegularization);

    for (const auto& params : testParameters)
    {
        TypeParam cost = this->loss->Cost(params);

        typename TestCategoricalCrossEntropy<TypeParam>::Vector probs;
        TypeParam sum = TypeParam(0.0f);

        for (std::size_t i = 0; i < this->NumberOfFeature; ++i)
        {
            probs[i] = TypeParam(std::exp(math::ToFloat(params[i])));
            sum += probs[i];
        }

        for (std::size_t i = 0; i < this->NumberOfFeature; ++i)
        {
            probs[i] = probs[i] / sum;
            probs[i] = TypeParam(std::max(std::min(math::ToFloat(probs[i]), 0.8f), 0.0001f));
        }

        TypeParam expectedCost = TypeParam(0.0f);
        for (std::size_t i = 0; i < this->NumberOfFeature; ++i)
            expectedCost += -this->target[i] * std::log(math::ToFloat(probs[i]));

        EXPECT_NEAR(math::ToFloat(cost), math::ToFloat(expectedCost + regValue), 0.1f);
    }
}
