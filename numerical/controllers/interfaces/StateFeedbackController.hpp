#pragma once

#include "numerical/math/Matrix.hpp"
#include "numerical/math/QNumber.hpp"

namespace controllers
{
    template<typename T, std::size_t StateSize, std::size_t InputSize>
    class StateFeedbackController
    {
        static_assert(math::detail::is_supported_type_v<T>,
            "StateFeedbackController only supports float or QNumber types");
        static_assert(math::detail::is_valid_dimensions_v<StateSize, StateSize>,
            "State dimensions must be positive");
        static_assert(math::detail::is_valid_dimensions_v<InputSize, InputSize>,
            "Input dimensions must be positive");

    public:
        using StateVector = math::Vector<T, StateSize>;
        using InputVector = math::Vector<T, InputSize>;

        virtual ~StateFeedbackController() = default;
        virtual InputVector ComputeControl(const StateVector& state) = 0;
    };
}
