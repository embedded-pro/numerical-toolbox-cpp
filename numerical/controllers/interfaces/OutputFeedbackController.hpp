#pragma once

#include "numerical/math/Matrix.hpp"
#include "numerical/math/QNumber.hpp"

namespace controllers
{
    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t MeasurementSize>
    class OutputFeedbackController
    {
        static_assert(math::detail::is_supported_type_v<T>,
            "OutputFeedbackController only supports float or QNumber types");
        static_assert(math::detail::is_valid_dimensions_v<StateSize, StateSize>,
            "State dimensions must be positive");
        static_assert(math::detail::is_valid_dimensions_v<InputSize, InputSize>,
            "Input dimensions must be positive");
        static_assert(math::detail::is_valid_dimensions_v<MeasurementSize, MeasurementSize>,
            "Measurement dimensions must be positive");

    public:
        using MeasurementVector = math::Vector<T, MeasurementSize>;
        using InputVector = math::Vector<T, InputSize>;

        virtual ~OutputFeedbackController() = default;
        virtual InputVector ComputeControl(const MeasurementVector& measurement) = 0;
    };
}
