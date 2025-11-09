#include "numerical/math/QNumber.hpp"
#include "numerical/neural_network/activation/ActivationFunction.hpp"
#include "numerical/neural_network/layer/Dense.hpp"
#include "numerical/neural_network/optimizer/GradientDescent.hpp"
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

    EXPECT_CALL(this->activation, Backward(T(0.5f)))
        .WillOnce(testing::Return(T(0.4f)));
    EXPECT_CALL(this->activation, Backward(T(0.7f)))
        .WillOnce(testing::Return(T(0.5f)));

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

TYPED_TEST(TestDense, OptimizationWithGradientDescent)
{
    using T = TypeParam;
    using ParameterVector = typename TestDense<T>::ParameterVector;

    constexpr size_t TotalParams = (TestDense<T>::InputSize * TestDense<T>::OutputSize) + TestDense<T>::OutputSize;

    neural_network::Dense<T, TestDense<T>::InputSize, TestDense<T>::OutputSize>
        dense(this->initialWeight, this->activation);

    MockLoss<T, TotalParams> loss;

    typename neural_network::GradientDescent<T, TotalParams>::Parameters params;
    params.learningRate = T(0.1f);
    params.maxIterations = 3;

    neural_network::GradientDescent<T, TotalParams> optimizer(params);

    auto initialParams = dense.Parameters();

    // Create gradient vectors
    ParameterVector gradVec1;
    ParameterVector gradVec2;
    ParameterVector gradVec3;

    for (std::size_t i = 0; i < TotalParams; ++i)
    {
        gradVec1[i] = T(0.5f);
        gradVec2[i] = T(0.4f);
        gradVec3[i] = T(0.3f);
    }

    {
        ::testing::InSequence seq;

        EXPECT_CALL(loss, Cost(testing::_))
            .WillOnce(testing::Return(T(1.0f)));

        EXPECT_CALL(loss, Gradient(testing::_))
            .WillOnce(testing::Return(gradVec1));

        EXPECT_CALL(loss, Cost(testing::_))
            .WillOnce(testing::Return(T(0.8f)));

        EXPECT_CALL(loss, Gradient(testing::_))
            .WillOnce(testing::Return(gradVec2));

        EXPECT_CALL(loss, Cost(testing::_))
            .WillOnce(testing::Return(T(0.6f)));

        EXPECT_CALL(loss, Gradient(testing::_))
            .WillOnce(testing::Return(gradVec3));

        EXPECT_CALL(loss, Cost(testing::_))
            .WillOnce(testing::Return(T(0.4f)));
    }

    auto result = optimizer.Minimize(initialParams, loss);

    // From your class definition, only verify iterations
    EXPECT_EQ(result.iterations, params.maxIterations);
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

        EXPECT_CALL(this->activation, Backward(outputGradient[0]))
            .WillOnce(testing::Return(T(0.3f)));
        EXPECT_CALL(this->activation, Backward(outputGradient[1]))
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

        EXPECT_CALL(this->activation, Backward(outputGradient1[0]))
            .WillOnce(testing::Return(T(0.3f)));
        EXPECT_CALL(this->activation, Backward(outputGradient1[1]))
            .WillOnce(testing::Return(T(0.4f)));

        EXPECT_CALL(this->activation, Forward(testing::_))
            .WillOnce(testing::Return(T(0.7f)));
        EXPECT_CALL(this->activation, Forward(testing::_))
            .WillOnce(testing::Return(T(0.8f)));

        EXPECT_CALL(this->activation, Backward(outputGradient2[0]))
            .WillOnce(testing::Return(T(0.5f)));
        EXPECT_CALL(this->activation, Backward(outputGradient2[1]))
            .WillOnce(testing::Return(T(0.6f)));
    }

    dense.Forward(input1);
    dense.Backward(outputGradient1);

    dense.Forward(input2);
    dense.Backward(outputGradient2);
}
