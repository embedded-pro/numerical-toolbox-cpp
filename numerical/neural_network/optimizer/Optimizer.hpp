#ifndef NEURAL_NETWORK_OPTIMIZER_HPP
#define NEURAL_NETWORK_OPTIMIZER_HPP

#include "numerical/math/QNumber.hpp"
#include "numerical/neural_network/losses/Loss.hpp"

namespace neural_network
{
    template<typename QNumberType, std::size_t NumberOfFeatures>
    class Optimizer
    {
        static_assert(math::is_qnumber<QNumberType>::value || std::is_floating_point<QNumberType>::value,
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

#endif
