#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/Matrix.hpp"
#include <cstddef>

namespace optimization
{
    template<std::size_t NumParams>
    class BlackBoxObjective
    {
    public:
        using ParameterVector = math::Vector<float, NumParams>;

        virtual ~BlackBoxObjective() = default;
        virtual float Evaluate(const ParameterVector& params) = 0;
    };
}
