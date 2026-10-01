#pragma once

#include "numerical/math/Matrix.hpp"

namespace optimization
{
    template<typename T, std::size_t N>
    class StepOptimizer
    {
        static_assert(std::is_floating_point_v<T>, "StepOptimizer supports floating-point types only");

    public:
        virtual ~StepOptimizer() = default;

        using Vector = math::Vector<T, N>;

        virtual void Step(Vector& theta, const Vector& gradient) = 0;
        virtual void Reset() = 0;
    };
}
