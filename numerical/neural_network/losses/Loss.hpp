#pragma once

#include "numerical/optimization/ObjectiveFunction.hpp"

namespace neural_network
{
    template<typename QNumberType, std::size_t NumberOfFeatures>
    class Loss
        : public optimization::ObjectiveFunction<QNumberType, NumberOfFeatures>
    {
        static_assert(math::is_qnumber_v<QNumberType> || std::is_floating_point_v<QNumberType>,
            "Loss can only be instantiated with math::QNumber types.");

    public:
        using Vector = math::Vector<QNumberType, NumberOfFeatures>;
    };
}
