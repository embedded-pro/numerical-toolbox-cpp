#pragma once

#include "numerical/math/Matrix.hpp"
#include "numerical/math/QNumber.hpp"
#include <array>

namespace controllers
{
    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t PredictionHorizon, std::size_t ControlHorizon = PredictionHorizon>
    class MpcController
    {
        static_assert(math::detail::is_supported_type_v<T>,
            "MpcController only supports float or QNumber types");
        static_assert(math::detail::is_valid_dimensions_v<StateSize, StateSize>,
            "State dimensions must be positive");
        static_assert(math::detail::is_valid_dimensions_v<InputSize, InputSize>,
            "Input dimensions must be positive");
        static_assert(PredictionHorizon > 0,
            "Prediction horizon must be positive");
        static_assert(ControlHorizon > 0 && ControlHorizon <= PredictionHorizon,
            "Control horizon must be positive and not exceed prediction horizon");

    public:
        using StateMatrix = math::SquareMatrix<T, StateSize>;
        using InputMatrix = math::Matrix<T, StateSize, InputSize>;
        using StateVector = math::Vector<T, StateSize>;
        using InputVector = math::Vector<T, InputSize>;
        using ControlSequence = std::array<InputVector, ControlHorizon>;

        virtual InputVector ComputeControl(const StateVector& state) = 0;
        [[nodiscard]] virtual const ControlSequence& GetControlSequence() const = 0;
    };
}
