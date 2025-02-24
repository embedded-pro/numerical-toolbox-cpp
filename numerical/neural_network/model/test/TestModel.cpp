#include "numerical/neural_network/layer/Layer.hpp"
#include "numerical/neural_network/model/Model.hpp"
#include "gmock/gmock.h"

namespace
{
    template<typename QNumberTypeMock, std::size_t InputSizeMock, std::size_t OutputSizeMock, std::size_t ParameterSizeMock>
    class LayerMock
        : public neural_network::Layer<QNumberTypeMock, InputSizeMock, OutputSizeMock, ParameterSizeMock>
    {
    public:
        using BaseLayer = neural_network::Layer<QNumberTypeMock, InputSizeMock, OutputSizeMock, ParameterSizeMock>;
        using InputVector = typename BaseLayer::InputVector;
        using OutputVector = typename BaseLayer::OutputVector;
        using ParameterVector = typename BaseLayer::ParameterVector;
        using QNumberType = QNumberTypeMock;

        static constexpr std::size_t InputSize = InputSizeMock;
        static constexpr std::size_t OutputSize = OutputSizeMock;
        static constexpr std::size_t ParameterSize = ParameterSizeMock;

        LayerMock()
        {
            setupDefaultBehavior();
        }

        LayerMock(const LayerMock& other)
            : inputGradient(other.inputGradient)
            , parameters(other.parameters)
        {
            setupDefaultBehavior();
        }

        LayerMock(LayerMock&& other) noexcept
            : inputGradient(std::move(other.inputGradient))
            , parameters(std::move(other.parameters))
        {
            setupDefaultBehavior();
        }

        LayerMock& operator=(const LayerMock&) = default;
        LayerMock& operator=(LayerMock&&) = default;

        MOCK_METHOD(void, Forward, (const InputVector&), (override));
        MOCK_METHOD(InputVector&, Backward, (const OutputVector&), (override));
        MOCK_METHOD(ParameterVector&, Parameters, (), (const, override));
        MOCK_METHOD(void, SetParameters, (const ParameterVector&), (override));

    private:
        void setupDefaultBehavior()
        {
            ON_CALL(*this, Forward(testing::_))
                .WillByDefault([this](const InputVector& input)
                    {
                        (void)input;
                    });

            ON_CALL(*this, Backward(testing::_))
                .WillByDefault([this](const OutputVector& output_gradient) -> InputVector&
                    {
                        (void)output_gradient;
                        return inputGradient;
                    });

            ON_CALL(*this, Parameters())
                .WillByDefault([this]() -> ParameterVector&
                    {
                        return parameters;
                    });

            ON_CALL(*this, SetParameters(testing::_))
                .WillByDefault([this](const ParameterVector& p)
                    {
                        parameters = p;
                    });
        }

        InputVector inputGradient;
        ParameterVector parameters;
    };

    template<typename T>
    class TestModel
        : public ::testing::Test
    {
    public:
        static constexpr std::size_t InputSize = 1;
        static constexpr std::size_t HiddenSize = 4;
        static constexpr std::size_t OutputSize = 1;
        static constexpr std::size_t ParameterSize = 2;

        using FirstLayer = LayerMock<T, InputSize, HiddenSize, ParameterSize>;
        using SecondLayer = LayerMock<T, HiddenSize, OutputSize, ParameterSize>;
        using ModelType = neural_network::Model<T, InputSize, OutputSize, FirstLayer, SecondLayer>;

        TestModel() = default;

        ModelType model;
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestModel, TestedTypes);
}

TYPED_TEST(TestModel, placeholder)
{
}
