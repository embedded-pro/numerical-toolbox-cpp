#include "numerical/neural_network/losses/BinaryCrossEntropy.hpp"
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
    class TestBinaryCrossEntropy
        : public ::testing::Test
    {
    public:
        static constexpr std::size_t NumberOfFeature = 2;
        using Vector = math::Matrix<T, NumberOfFeature, 1>;

        MockRegularization<T, NumberOfFeature> mockRegularization;
        Vector target;
        std::optional<neural_network::BinaryCrossEntropy<T, NumberOfFeature>> loss;

        void SetUp() override
        {
            target = Vector{ T(0.8f), T(0.2f) };
        }
    };

    using TestedTypes = ::testing::Types<float>;
    TYPED_TEST_SUITE(TestBinaryCrossEntropy, TestedTypes);
}

TYPED_TEST(TestBinaryCrossEntropy, CostCallsRegularization)
{
    typename TestBinaryCrossEntropy<TypeParam>::Vector parameters{ TypeParam(0.7f), TypeParam(0.3f) };
    TypeParam regValue = TypeParam(0.1f);

    EXPECT_CALL(this->mockRegularization, Calculate(::testing::_))
        .WillOnce(::testing::Return(regValue));

    this->loss.emplace(this->target, this->mockRegularization);
    TypeParam cost = this->loss->Cost(parameters);

    TypeParam expectedCost = TypeParam(0.0f);
    for (std::size_t i = 0; i < this->NumberOfFeature; ++i)
    {
        TypeParam pred = std::max(std::min(parameters[i], TypeParam(0.9999f)), TypeParam(0.0001f));
        expectedCost += -(this->target[i] * std::log(math::ToFloat(pred)) +
                          (TypeParam(0.9999f) - this->target[i]) * std::log(math::ToFloat(TypeParam(0.9999f) - pred)));
    }

    EXPECT_NEAR(math::ToFloat(cost), math::ToFloat(expectedCost + regValue), 0.1f);
}

TYPED_TEST(TestBinaryCrossEntropy, PerfectPrediction)
{
    typename TestBinaryCrossEntropy<TypeParam>::Vector parameters = this->target;
    TypeParam regValue = TypeParam(0.0f);

    EXPECT_CALL(this->mockRegularization, Calculate(::testing::_))
        .WillOnce(::testing::Return(regValue));

    this->loss.emplace(this->target, this->mockRegularization);
    TypeParam cost = this->loss->Cost(parameters);

    // Due to clamping, perfect prediction won't reach exactly zero
    EXPECT_GT(math::ToFloat(cost), 0.0f);
    EXPECT_LT(math::ToFloat(cost), 1.1f);
}

TYPED_TEST(TestBinaryCrossEntropy, WorstPrediction)
{
    typename TestBinaryCrossEntropy<TypeParam>::Vector parameters{ TypeParam(0.1f), TypeParam(0.9f) };
    TypeParam regValue = TypeParam(0.0f);

    EXPECT_CALL(this->mockRegularization, Calculate(::testing::_))
        .WillOnce(::testing::Return(regValue));

    this->loss.emplace(this->target, this->mockRegularization);
    TypeParam cost = this->loss->Cost(parameters);

    // Worst prediction should have significantly higher cost than best prediction
    EXPECT_GT(math::ToFloat(cost), 1.0f);
}

TYPED_TEST(TestBinaryCrossEntropy, GradientTest)
{
    typename TestBinaryCrossEntropy<TypeParam>::Vector parameters{ TypeParam(0.6f), TypeParam(0.4f) };
    TypeParam regValue = TypeParam(0.1f);

    EXPECT_CALL(this->mockRegularization, Calculate(::testing::_))
        .Times(this->NumberOfFeature)
        .WillRepeatedly(::testing::Return(regValue));

    this->loss.emplace(this->target, this->mockRegularization);
    auto gradient = this->loss->Gradient(parameters);

    for (std::size_t i = 0; i < this->NumberOfFeature; ++i)
    {
        TypeParam pred = std::max(std::min(parameters[i], TypeParam(0.9999f)), TypeParam(0.0001f));
        TypeParam expectedGrad = (pred - this->target[i]) / (pred * (TypeParam(0.9999f) - pred)) + regValue;
        EXPECT_NEAR(math::ToFloat(gradient[i]), math::ToFloat(expectedGrad), 0.15f);
    }
}

TYPED_TEST(TestBinaryCrossEntropy, MultipleParameters)
{
    std::array<typename TestBinaryCrossEntropy<TypeParam>::Vector, 2> testParameters = {
        typename TestBinaryCrossEntropy<TypeParam>::Vector{ TypeParam(0.3f), TypeParam(0.7f) },
        typename TestBinaryCrossEntropy<TypeParam>::Vector{ TypeParam(0.9f), TypeParam(0.1f) }
    };

    TypeParam regValue = TypeParam(0.2f);

    EXPECT_CALL(this->mockRegularization, Calculate(::testing::_))
        .Times(testParameters.size())
        .WillRepeatedly(::testing::Return(regValue));

    this->loss.emplace(this->target, this->mockRegularization);

    for (const auto& params : testParameters)
    {
        TypeParam cost = this->loss->Cost(params);

        TypeParam expectedCost = TypeParam(0.0f);
        for (std::size_t i = 0; i < this->NumberOfFeature; ++i)
        {
            TypeParam pred = std::max(std::min(params[i], TypeParam(0.9999f)), TypeParam(0.0001f));
            expectedCost += -(this->target[i] * std::log(math::ToFloat(pred)) +
                              (TypeParam(0.9999f) - this->target[i]) * std::log(math::ToFloat(TypeParam(0.9999f) - pred)));
        }

        EXPECT_NEAR(math::ToFloat(cost), math::ToFloat(expectedCost + regValue), 0.1f);
    }
}
