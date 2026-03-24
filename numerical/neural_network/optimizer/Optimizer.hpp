#pragma once

#include "numerical/math/QNumber.hpp"
#include "numerical/neural_network/losses/Loss.hpp"

namespace neural_network
{
    template<typename QNumberType, std::size_t NumberOfFeatures>
    class Optimizer
    {
        static_assert(math::is_qnumber_v<QNumberType> || std::is_floating_point_v<QNumberType>,
            "Optimizer can only be instantiated with math::QNumber types.");

    public:
        using Vector = typename Loss<QNumberType, NumberOfFeatures>::Vector;

        struct Result
        {
            Result(Vector parameters, QNumberType finalCost, size_t iterations)
                : parameters(parameters)
                , finalCost(finalCost)
                , iterations(iterations)
            {}

            Vector parameters;
            QNumberType finalCost;
            std::size_t iterations;
        };

        virtual const Result& Minimize(const Vector& initialGuess, Loss<QNumberType, NumberOfFeatures>& loss) = 0;
    };
}
