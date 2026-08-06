#pragma once

#include "numerical/math/Matrix.hpp"

namespace optimization
{
    template<typename QNumberType, std::size_t NumberOfFeatures>
    class ObjectiveFunction
    {
        static_assert(math::is_qnumber_v<QNumberType> || std::is_floating_point_v<QNumberType>,
            "ObjectiveFunction can only be instantiated with math::QNumber types.");

    public:
        using Vector = math::Vector<QNumberType, NumberOfFeatures>;

        virtual QNumberType Cost(const Vector& parameters) = 0;
        virtual Vector Gradient(const Vector& parameters) = 0;
        virtual ~ObjectiveFunction() = default;
    };
}
