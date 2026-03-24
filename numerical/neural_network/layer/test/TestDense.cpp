#include "numerical/math/QNumber.hpp"
#include "numerical/neural_network/activation/ActivationFunction.hpp"
#include "numerical/neural_network/layer/Dense.hpp"
#include "numerical/neural_network/optimizer/Optimizer.hpp"
#include "gmock/gmock.h"

namespace
{
    template<typename T>
    class ActivationMock
        : public neural_network::ActivationFunction<T>
    {
    public:
        MOCK_METHOD(T, Forward, (T x), (const, override));
        MOCK_METHOD(T, Backward, (T x), (const, override));
    };

    template<typename T, std::size_t Features>
    class MockLoss
        : public neural_network::Loss<T, Features>
    {
    public:
        using Vector = typename neural_network::Loss<T, Features>::Vector;

        MOCK_METHOD(T, Cost, (const Vector& parameters), (override));
        MOCK_METHOD(Vector, Gradient, (const Vector& parameters), (override));
    };

    template<typename T, std::size_t Features>
    class MockOptimizer
        : public neural_network::Optimizer<T, Features>
    {
    public:
        using Result = typename neural_network::Optimizer<T, Features>::Result;
        using Vector = typename neural_network::Loss<T, Features>::Vector;

        const Result& Minimize(const Vector& initialGuess, neural_network::Loss<T, Features>& loss) override
        {
            return result;
        }

        Result result{ Vector{}, T(0), 0 };
    };

    template<typename T>
    class TestDense
        : public ::testing::Test
    {
    public:
        static constexpr std::size_t InputSize = 3;
        static constexpr std::size_t OutputSize = 2;
        using WeightMatrix = math::Matrix<T, OutputSize, InputSize>;
        using InputVector = math::Vector<T, InputSize>;
        using OutputVector = math::Vector<T, OutputSize>;
        using ParameterVector = math::Vector<T, (InputSize * OutputSize) + OutputSize>;

        void SetUp() override
        {
            for (std::size_t i = 0; i < OutputSize; ++i)
                for (std::size_t j = 0; j < InputSize; ++j)
                    initialWeight.at(i, j) = T(0.1f);
        }

        WeightMatrix initialWeight;
        ActivationMock<T> activation;
    };

    using TestedTypes = ::testing::Types<float>;
    TYPED_TEST_SUITE(TestDense, TestedTypes);
}

TYPED_TEST(TestDense, Construction)
{
    using T = TypeParam;
    neural_network::Dense<T, TestDense<T>::InputSize, TestDense<T>::OutputSize>
        dense(this->initialWeight, this->activation);

    auto params = dense.Parameters();

    std::size_t idx = 0;
    for (std::size_t i = 0; i < TestDense<T>::OutputSize; ++i)
    {
        for (std::size_t j = 0; j < TestDense<T>::InputSize; ++j)
        {
            EXPECT_EQ(params[idx++], this->initialWeight.at(i, j));
        }
    }

    for (std::size_t i = 0; i < TestDense<T>::OutputSize; ++i)
    {
        EXPECT_EQ(params[idx++], T(0.0f));
    }
}

TYPED_TEST(TestDense, ForwardPropagation)
{
    using T = TypeParam;
    using InputVector = typename TestDense<T>::InputVector;

    neural_network::Dense<T, TestDense<T>::InputSize, TestDense<T>::OutputSize>
        dense(this->initialWeight, this->activation);

    InputVector input;
    input[0] = T(1.0f);
    input[1] = T(2.0f);
    input[2] = T(3.0f);

    T expectedPreActivation = T(0.6f);

    EXPECT_CALL(this->activation, Forward(expectedPreActivation))
        .Times(TestDense<T>::OutputSize)
        .WillRepeatedly(testing::Return(T(0.5f)));

    dense.Forward(input);
}

TYPED_TEST(TestDense, BackwardPropagation)
{
    using T = TypeParam;
    using InputVector = typename TestDense<T>::InputVector;
    using OutputVector = typename TestDense<T>::OutputVector;

    neural_network::Dense<T, TestDense<T>::InputSize, TestDense<T>::OutputSize>
        dense(this->initialWeight, this->activation);

    InputVector input;
    input[0] = T(1.0f);
    input[1] = T(2.0f);
    input[2] = T(3.0f);

    OutputVector outputGradient;
    outputGradient[0] = T(0.5f);
    outputGradient[1] = T(0.7f);

    T expectedPreActivation = T(0.6f);
    EXPECT_CALL(this->activation, Forward(expectedPreActivation))
        .Times(TestDense<T>::OutputSize)
        .WillRepeatedly(testing::Return(T(0.5f)));

    dense.Forward(input);

    T expectedPreActivationBackward = T(0.6f);
    EXPECT_CALL(this->activation, Backward(expectedPreActivationBackward))
        .Times(TestDense<T>::OutputSize)
        .WillRepeatedly(testing::Return(T(0.4f)));

    dense.Backward(outputGradient);
}

TYPED_TEST(TestDense, ParametersExtractionAndSetting)
{
    using T = TypeParam;
    using ParameterVector = typename TestDense<T>::ParameterVector;

    neural_network::Dense<T, TestDense<T>::InputSize, TestDense<T>::OutputSize>
        dense(this->initialWeight, this->activation);

    auto params = dense.Parameters();

    ParameterVector newParams;
    for (std::size_t i = 0; i < ParameterVector::size; ++i)
    {
        newParams[i] = T(0.2f);
    }

    dense.SetParameters(newParams);

    auto updatedParams = dense.Parameters();
    for (std::size_t i = 0; i < ParameterVector::size; ++i)
    {
        EXPECT_EQ(updatedParams[i], newParams[i]);
    }
}

TYPED_TEST(TestDense, OptimizationWithMockedOptimizer)
{
    using T = TypeParam;
    using ParameterVector = typename TestDense<T>::ParameterVector;

    constexpr size_t TotalParams = (TestDense<T>::InputSize * TestDense<T>::OutputSize) + TestDense<T>::OutputSize;

    neural_network::Dense<T, TestDense<T>::InputSize, TestDense<T>::OutputSize>
        dense(this->initialWeight, this->activation);

    MockOptimizer<T, TotalParams> optimizer;

    auto initialParams = dense.Parameters();

    ParameterVector optimizedParams;
    for (std::size_t i = 0; i < TotalParams; ++i)
        optimizedParams[i] = T(0.05f);

    optimizer.result = typename neural_network::Optimizer<T, TotalParams>::Result(
        optimizedParams, T(0.4f), 3);

    MockLoss<T, TotalParams> loss;
    auto& result = optimizer.Minimize(initialParams, loss);

    dense.SetParameters(result.parameters);

    auto updatedParams = dense.Parameters();
    for (std::size_t i = 0; i < TotalParams; ++i)
        EXPECT_EQ(updatedParams[i], optimizedParams[i]);

    EXPECT_EQ(result.iterations, 3u);
}

TYPED_TEST(TestDense, FullLayerSequence)
{
    using T = TypeParam;
    using InputVector = typename TestDense<T>::InputVector;
    using OutputVector = typename TestDense<T>::OutputVector;

    neural_network::Dense<T, TestDense<T>::InputSize, TestDense<T>::OutputSize>
        dense(this->initialWeight, this->activation);

    InputVector input;
    input[0] = T(1.0f);
    input[1] = T(2.0f);
    input[2] = T(3.0f);

    OutputVector outputGradient;
    outputGradient[0] = T(0.5f);
    outputGradient[1] = T(0.7f);

    {
        ::testing::InSequence seq;

        EXPECT_CALL(this->activation, Forward(testing::_))
            .WillOnce(testing::Return(T(0.5f)));
        EXPECT_CALL(this->activation, Forward(testing::_))
            .WillOnce(testing::Return(T(0.6f)));

        EXPECT_CALL(this->activation, Backward(testing::_))
            .WillOnce(testing::Return(T(0.3f)));
        EXPECT_CALL(this->activation, Backward(testing::_))
            .WillOnce(testing::Return(T(0.4f)));
    }

    dense.Forward(input);
    dense.Backward(outputGradient);
}

TYPED_TEST(TestDense, MultipleForwardBackwardPasses)
{
    using T = TypeParam;
    using InputVector = typename TestDense<T>::InputVector;
    using OutputVector = typename TestDense<T>::OutputVector;

    neural_network::Dense<T, TestDense<T>::InputSize, TestDense<T>::OutputSize>
        dense(this->initialWeight, this->activation);

    InputVector input1, input2;
    input1[0] = T(1.0f);
    input1[1] = T(2.0f);
    input1[2] = T(3.0f);
    input2[0] = T(4.0f);
    input2[1] = T(5.0f);
    input2[2] = T(6.0f);

    OutputVector outputGradient1, outputGradient2;
    outputGradient1[0] = T(0.1f);
    outputGradient1[1] = T(0.2f);
    outputGradient2[0] = T(0.3f);
    outputGradient2[1] = T(0.4f);

    {
        ::testing::InSequence seq;

        EXPECT_CALL(this->activation, Forward(testing::_))
            .WillOnce(testing::Return(T(0.5f)));
        EXPECT_CALL(this->activation, Forward(testing::_))
            .WillOnce(testing::Return(T(0.6f)));

        EXPECT_CALL(this->activation, Backward(testing::_))
            .WillOnce(testing::Return(T(0.3f)));
        EXPECT_CALL(this->activation, Backward(testing::_))
            .WillOnce(testing::Return(T(0.4f)));

        EXPECT_CALL(this->activation, Forward(testing::_))
            .WillOnce(testing::Return(T(0.7f)));
        EXPECT_CALL(this->activation, Forward(testing::_))
            .WillOnce(testing::Return(T(0.8f)));

        EXPECT_CALL(this->activation, Backward(testing::_))
            .WillOnce(testing::Return(T(0.5f)));
        EXPECT_CALL(this->activation, Backward(testing::_))
            .WillOnce(testing::Return(T(0.6f)));
    }

    dense.Forward(input1);
    dense.Backward(outputGradient1);

    dense.Forward(input2);
    dense.Backward(outputGradient2);
}
