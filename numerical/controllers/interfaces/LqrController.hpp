#pragma once

#include "numerical/math/Matrix.hpp"
#include "numerical/math/QNumber.hpp"

namespace controllers
{
    template<typename T, std::size_t StateSize, std::size_t InputSize>
    class LqrController
    {
        static_assert(math::detail::is_supported_type_v<T>,
            "LqrController only supports float or QNumber types");
        static_assert(math::detail::is_valid_dimensions_v<StateSize, StateSize>,
            "State dimensions must be positive");
        static_assert(math::detail::is_valid_dimensions_v<InputSize, InputSize>,
            "Input dimensions must be positive");

    public:
        using StateMatrix = math::SquareMatrix<T, StateSize>;
        using InputMatrix = math::Matrix<T, StateSize, InputSize>;
        using StateVector = math::Vector<T, StateSize>;
        using InputVector = math::Vector<T, InputSize>;
        using GainMatrix = math::Matrix<T, InputSize, StateSize>;

        virtual InputVector ComputeControl(const StateVector& state) = 0;
        [[nodiscard]] virtual const GainMatrix& GetGain() const = 0;
    };
}
