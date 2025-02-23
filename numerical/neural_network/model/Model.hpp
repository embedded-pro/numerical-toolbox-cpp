#ifndef NEURAL_NETWORK_MODEL_HPP
#define NEURAL_NETWORK_MODEL_HPP

#include "numerical/neural_network/Layer.hpp"
#include "optimizers/Loss.hpp"
#include "optimizers/Optimizer.hpp"
#include <tuple>

namespace neural_network
{
    namespace detail
    {
        template<typename QNumberType, typename T>
        struct is_layer_impl
        {
            template<typename L>
            static constexpr bool check()
            {
                return std::is_base_of<
                    Layer<QNumberType,
                        L::InputSize,
                        L::OutputSize,
                        L::ParameterSize>,
                    L>::value;
            }
        };

        template<typename QNumberType, typename T>
        struct is_layer : std::integral_constant<bool, is_layer_impl<QNumberType, T>::template check<T>()>
        {};

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
            static constexpr bool value =
                (InputSize == First::InputSize) &&
                verify_layer_sizes<First::OutputSize, Rest...>::value;
        };
    }

    template<typename QNumberType, std::size_t InputSize, std::size_t OutputSize, typename... Layers>
    class Model
    {
        static_assert(math::is_qnumber<QNumberType>::value || std::is_floating_point<QNumberType>::value,
            "Model can only be instantiated with math::QNumber types.");

        static_assert(detail::all_are_layers<QNumberType, Layers...>::value,
            "All types in Layers must derive from Layer");

        static_assert(detail::verify_layer_sizes<InputSize, Layers...>::value &&
                          (sizeof...(Layers) == 0 ||
                              std::tuple_element_t<sizeof...(Layers) - 1, std::tuple<Layers...>>::OutputSize == OutputSize),
            "Layer sizes do not match");

    public:
        using InputVector = math::Vector<QNumberType, InputSize>;
        using OutputVector = math::Vector<QNumberType, OutputSize>;
        static constexpr std::size_t TotalParameters = (Layers::ParameterSize + ...);

        Model();

        OutputVector Forward(const InputVector& input);
        InputVector Backward(const OutputVector& output_gradient);
        void Train(optimizer::Optimizer<QNumberType, TotalParameters>& optimizer, optimizer::Loss<QNumberType, TotalParameters>& loss, const math::Vector<QNumberType, TotalParameters>& initialParameters);
        void SetParameters(const math::Vector<QNumberType, TotalParameters>& parameters);
        math::Vector<QNumberType, TotalParameters> GetParameters() const;

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
        math::Vector<QNumberType, TotalParameters> GetParametersImpl(std::index_sequence<Is...>) const;

        template<typename Layer>
        void GetLayerParameters(const Layer& layer, math::Vector<QNumberType, TotalParameters>& parameters, std::size_t& offset) const;

        std::tuple<Layers...> layers;
        InputVector currentInput;
    };

    // Implementation //

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
    void Model<QNumberType, InputSize, OutputSize, Layers...>::Train(optimizer::Optimizer<QNumberType, TotalParameters>& optimizer, optimizer::Loss<QNumberType, TotalParameters>& loss, const math::Vector<QNumberType, TotalParameters>& initialParameters)
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
    typename math::Vector<QNumberType, Model<QNumberType, InputSize, OutputSize, Layers...>::TotalParameters> Model<QNumberType, InputSize, OutputSize, Layers...>::GetParameters() const
    {
        return GetParametersImpl(std::make_index_sequence<sizeof...(Layers)>{});
    }

    template<typename QNumberType, std::size_t InputSize, std::size_t OutputSize, typename... Layers>
    template<std::size_t... Is>
    typename Model<QNumberType, InputSize, OutputSize, Layers...>::OutputVector
    Model<QNumberType, InputSize, OutputSize, Layers...>::ForwardImpl(const InputVector& input, std::index_sequence<Is...>)
    {
        auto current = input;
        int dummy[] = { 0, ((current = std::get<Is>(layers).Forward(current)), 0)... };
        (void)dummy;
        return current;
    }

    template<typename QNumberType, std::size_t InputSize, std::size_t OutputSize, typename... Layers>
    template<std::size_t... Is>
    typename Model<QNumberType, InputSize, OutputSize, Layers...>::InputVector
    Model<QNumberType, InputSize, OutputSize, Layers...>::BackwardImpl(const OutputVector& output_gradient, std::index_sequence<Is...>)
    {
        auto current_gradient = output_gradient;
        int dummy[] = { 0, ((current_gradient = std::get<sizeof...(Layers) - 1 - Is>(layers).Backward(current_gradient)), 0)... };
        (void)dummy;
        return current_gradient;
    }

    template<typename QNumberType, std::size_t InputSize, std::size_t OutputSize, typename... Layers>
    template<std::size_t... Is>
    void Model<QNumberType, InputSize, OutputSize, Layers...>::SetParametersImpl(
        const math::Vector<QNumberType, TotalParameters>& parameters,
        std::index_sequence<Is...>)
    {
        std::size_t offset = 0;
        int dummy[] = { 0, (SetLayerParameters(std::get<Is>(layers), parameters, offset), 0)... };
        (void)dummy;
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
