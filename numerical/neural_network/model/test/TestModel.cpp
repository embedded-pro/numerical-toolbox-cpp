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

        LayerMock(int mockParam = 0)
        {
            (void)mockParam;
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

        TestModel()
            : model(neural_network::make_layer<FirstLayer>(42),
                  neural_network::make_layer<SecondLayer>(43))
        {}

        ModelType model;
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestModel, TestedTypes);
}

TYPED_TEST(TestModel, LayerConstructionWithParameters)
{
    using FirstLayer = typename TestFixture::FirstLayer;
    using SecondLayer = typename TestFixture::SecondLayer;

    auto firstLayerParam = 100;
    auto secondLayerParam = 200;

    typename TestFixture::ModelType customModel(
        neural_network::make_layer<FirstLayer>(firstLayerParam),
        neural_network::make_layer<SecondLayer>(secondLayerParam));
}

TYPED_TEST(TestModel, SetParameters)
{
    using ParameterVector = math::Vector<TypeParam, TestFixture::ModelType::TotalParameters>;

    ParameterVector parameters;
    for (size_t i = 0; i < TestFixture::ModelType::TotalParameters; ++i)
        parameters[i] = TypeParam(static_cast<float>(i) * 0.1f);

    TestFixture::model.SetParameters(parameters);

    auto retrievedParams = TestFixture::model.GetParameters();

    for (size_t i = 0; i < TestFixture::ModelType::TotalParameters; ++i)
        EXPECT_EQ(math::ToFloat(retrievedParams[i]), math::ToFloat(parameters[i]));
}

TYPED_TEST(TestModel, ConstructorVariants)
{
    typename TestFixture::ModelType defaultModel;

    typename TestFixture::ModelType factoryModel(
        neural_network::make_layer<typename TestFixture::FirstLayer>(1),
        neural_network::make_layer<typename TestFixture::SecondLayer>(2));

    typename TestFixture::ModelType::InputVector input;
    auto defaultOutput = defaultModel.Forward(input);
    auto factoryOutput = factoryModel.Forward(input);

    typename TestFixture::ModelType::OutputVector outputGradient;
    auto defaultGradient = defaultModel.Backward(outputGradient);
    auto factoryGradient = factoryModel.Backward(outputGradient);
}
