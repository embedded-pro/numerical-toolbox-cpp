#ifndef NEURAL_NETWORK_MODEL_HPP
#define NEURAL_NETWORK_MODEL_HPP

#include "infra/util/BoundedVector.hpp"
#include "numerical/neural_network/layer/Layer.hpp"
#include "numerical/neural_network/losses/Loss.hpp"
#include "numerical/neural_network/optimizer/Optimizer.hpp"
#include <functional>
#include <tuple>
#include <utility>

namespace neural_network
{
    template<typename Layer, typename... Args>
    auto make_layer(Args&&... args)
    {
        return [args = std::make_tuple(std::forward<Args>(args)...)]() mutable
        {
            return std::apply([](auto&&... params)
                {
                    return Layer(std::forward<decltype(params)>(params)...);
                },
                std::move(args));
        };
    }

    namespace detail
    {
        template<typename QNumberType, typename T>
        struct is_layer
        {
            static constexpr bool value = std::is_base_of_v<
                Layer<QNumberType,
                    T::InputSize,
                    T::OutputSize,
                    T::ParameterSize>,
                T>;
        };

        template<typename QNumberType, typename... Layers>
        struct all_are_layers;

        template<typename QNumberType>
        struct all_are_layers<QNumberType> : std::true_type
        {};

        template<typename QNumberType, typename First, typename... Rest>
        struct all_are_layers<QNumberType, First, Rest...>
            : std::integral_constant<bool,
                  is_layer<QNumberType, First>::value &&
                      all_are_layers<QNumberType, Rest...>::value>
        {};

        template<std::size_t InputSize, typename... Layers>
        struct verify_layer_sizes;

        template<std::size_t InputSize>
        struct verify_layer_sizes<InputSize>
        {
            static constexpr bool value = true;
        };

        template<std::size_t InputSize, typename First, typename... Rest>
        struct verify_layer_sizes<InputSize, First, Rest...>
        {
            using FirstLayer = Layer<typename First::QNumberType,
                First::InputSize,
                First::OutputSize,
                First::ParameterSize>;

            static constexpr bool value =
                (InputSize == FirstLayer::InputSize) &&
                verify_layer_sizes<FirstLayer::OutputSize, Rest...>::value;
        };

        template<typename QNumberType, typename... Layers>
        constexpr std::size_t calculate_total_parameters()
        {
            return (... + Layers::ParameterSize);
        }
    }

    template<typename QNumberType, std::size_t InputSize, std::size_t OutputSize, typename... Layers>
    class Model
    {
        static_assert(math::is_qnumber<QNumberType>::value || std::is_floating_point<QNumberType>::value,
            "Model can only be instantiated with math::QNumber types or floating point types.");

        static_assert(detail::all_are_layers<QNumberType, Layers...>::value,
            "All types in Layers must derive from Layer");

        static_assert(sizeof...(Layers) > 0, "Model must have at least one layer");

        static_assert(detail::verify_layer_sizes<InputSize, Layers...>::value &&
                          std::tuple_element_t<sizeof...(Layers) - 1,
                              std::tuple<Layers...>>::OutputSize == OutputSize,
            "Layer sizes do not match");

    public:
        using InputVector = math::Matrix<QNumberType, InputSize, 1>;
        using OutputVector = math::Matrix<QNumberType, OutputSize, 1>;
        static constexpr std::size_t TotalParameters = detail::calculate_total_parameters<QNumberType, Layers...>();

        Model();

        template<typename... FactoryFuncs,
            typename = std::enable_if_t<sizeof...(FactoryFuncs) == sizeof...(Layers)>>
        Model(FactoryFuncs&&... factories)
            : layers(std::invoke(std::forward<FactoryFuncs>(factories))...)
        {}

        OutputVector Forward(const InputVector& input);
        InputVector Backward(const OutputVector& output_gradient);
        void Train(Optimizer<QNumberType, TotalParameters>& optimizer, Loss<QNumberType, TotalParameters>& loss, const math::Vector<QNumberType, TotalParameters>& initialParameters);
        void SetParameters(const math::Vector<QNumberType, TotalParameters>& parameters);
        math::Vector<QNumberType, Model::TotalParameters> GetParameters() const;

    private:
        template<std::size_t... Is>
        OutputVector ForwardImpl(const InputVector& input, std::index_sequence<Is...>);

        template<std::size_t... Is>
        InputVector BackwardImpl(const OutputVector& output_gradient, std::index_sequence<Is...>);

        template<std::size_t... Is>
        void SetParametersImpl(const math::Vector<QNumberType, TotalParameters>& parameters, std::index_sequence<Is...>);

        template<typename Layer>
        void SetLayerParameters(Layer& layer, const math::Vector<QNumberType, TotalParameters>& parameters, std::size_t& offset);

        template<std::size_t... Is>
        math::Vector<QNumberType, Model::TotalParameters> GetParametersImpl(std::index_sequence<Is...>) const;

        template<typename Layer>
        void GetLayerParameters(const Layer& layer, math::Vector<QNumberType, TotalParameters>& parameters, std::size_t& offset) const;

        std::tuple<Layers...> layers;
        InputVector currentInput;
    };

    template<typename QNumberType, std::size_t InputSize, std::size_t OutputSize, typename... Layers>
    Model<QNumberType, InputSize, OutputSize, Layers...>::Model()
        : layers(std::make_tuple(Layers()...))
    {}

    template<typename QNumberType, std::size_t InputSize, std::size_t OutputSize, typename... Layers>
    typename Model<QNumberType, InputSize, OutputSize, Layers...>::OutputVector Model<QNumberType, InputSize, OutputSize, Layers...>::Forward(const InputVector& input)
    {
        currentInput = input;
        return ForwardImpl(input, std::make_index_sequence<sizeof...(Layers)>{});
    }

    template<typename QNumberType, std::size_t InputSize, std::size_t OutputSize, typename... Layers>
    typename Model<QNumberType, InputSize, OutputSize, Layers...>::InputVector Model<QNumberType, InputSize, OutputSize, Layers...>::Backward(const OutputVector& output_gradient)
    {
        return BackwardImpl(output_gradient, std::make_index_sequence<sizeof...(Layers)>{});
    }

    template<typename QNumberType, std::size_t InputSize, std::size_t OutputSize, typename... Layers>
    void Model<QNumberType, InputSize, OutputSize, Layers...>::Train(Optimizer<QNumberType, TotalParameters>& optimizer, Loss<QNumberType, TotalParameters>& loss, const math::Vector<QNumberType, TotalParameters>& initialParameters)
    {
        auto result = optimizer.Minimize(initialParameters, loss);
        SetParameters(result.parameters);
    }

    template<typename QNumberType, std::size_t InputSize, std::size_t OutputSize, typename... Layers>
    void Model<QNumberType, InputSize, OutputSize, Layers...>::SetParameters(const math::Vector<QNumberType, TotalParameters>& parameters)
    {
        SetParametersImpl(parameters, std::make_index_sequence<sizeof...(Layers)>{});
    }

    template<typename QNumberType, std::size_t InputSize, std::size_t OutputSize, typename... Layers>
    math::Vector<QNumberType, Model<QNumberType, InputSize, OutputSize, Layers...>::TotalParameters> Model<QNumberType, InputSize, OutputSize, Layers...>::GetParameters() const
    {
        return GetParametersImpl(std::make_index_sequence<sizeof...(Layers)>{});
    }

    template<typename QNumberType, std::size_t InputSize, std::size_t OutputSize, typename... Layers>
    template<std::size_t... Is>
    typename Model<QNumberType, InputSize, OutputSize, Layers...>::OutputVector
    Model<QNumberType, InputSize, OutputSize, Layers...>::ForwardImpl(const InputVector& input, std::index_sequence<Is...>)
    {
        return OutputVector();
    }

    template<typename QNumberType, std::size_t InputSize, std::size_t OutputSize, typename... Layers>
    template<std::size_t... Is>
    typename Model<QNumberType, InputSize, OutputSize, Layers...>::InputVector
    Model<QNumberType, InputSize, OutputSize, Layers...>::BackwardImpl(const OutputVector& output_gradient, std::index_sequence<Is...>)
    {
        return InputVector();
    }

    template<typename QNumberType, std::size_t InputSize, std::size_t OutputSize, typename... Layers>
    template<std::size_t... Is>
    void Model<QNumberType, InputSize, OutputSize, Layers...>::SetParametersImpl(
        const math::Vector<QNumberType, TotalParameters>& parameters,
        std::index_sequence<Is...>)
    {
        std::size_t offset = 0;
        (SetLayerParameters(std::get<Is>(layers), parameters, offset), ...);
    }

    template<typename QNumberType, std::size_t InputSize, std::size_t OutputSize, typename... Layers>
    template<typename Layer>
    void Model<QNumberType, InputSize, OutputSize, Layers...>::SetLayerParameters(
        Layer& layer,
        const math::Vector<QNumberType, TotalParameters>& parameters,
        std::size_t& offset)
    {
        const std::size_t layerParameterSize = Layer::ParameterSize;
        math::Vector<QNumberType, layerParameterSize> layerParameters;

        for (std::size_t i = 0; i < layerParameterSize; ++i)
            layerParameters[i] = parameters[offset + i];

        layer.SetParameters(layerParameters);
        offset += layerParameterSize;
    }

    template<typename QNumberType, std::size_t InputSize, std::size_t OutputSize, typename... Layers>
    template<std::size_t... Is>
    math::Vector<QNumberType, Model<QNumberType, InputSize, OutputSize, Layers...>::TotalParameters>
    Model<QNumberType, InputSize, OutputSize, Layers...>::GetParametersImpl(
        std::index_sequence<Is...>) const
    {
        math::Vector<QNumberType, TotalParameters> parameters;
        std::size_t offset = 0;
        int dummy[] = { 0, (GetLayerParameters(std::get<Is>(layers), parameters, offset), 0)... };
        (void)dummy;
        return parameters;
    }

    template<typename QNumberType, std::size_t InputSize, std::size_t OutputSize, typename... Layers>
    template<typename Layer>
    void Model<QNumberType, InputSize, OutputSize, Layers...>::GetLayerParameters(const Layer& layer, math::Vector<QNumberType, TotalParameters>& parameters, std::size_t& offset) const
    {
        const auto& layerParameters = layer.Parameters();
        for (std::size_t i = 0; i < Layer::ParameterSize; ++i)
            parameters[offset + i] = layerParameters[i];
        offset += Layer::ParameterSize;
    }
}

#endif
