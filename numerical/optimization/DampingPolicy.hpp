#pragma once

#include "numerical/math/Matrix.hpp"
#include <cstddef>
#include <type_traits>

namespace optimization
{
    template<typename T, std::size_t Residuals, std::size_t Parameters>
    class DampingPolicy
    {
        static_assert(std::is_floating_point_v<T>, "DampingPolicy supports floating-point types only");

    public:
        [[nodiscard]] virtual T DampingSquared(const math::Matrix<T, Residuals, Parameters>& jacobian) const = 0;

    protected:
        ~DampingPolicy() = default;
    };

    template<typename T, std::size_t Residuals, std::size_t Parameters>
    class ConstantDamping
        : public DampingPolicy<T, Residuals, Parameters>
    {
    public:
        explicit ConstantDamping(T lambdaSquared)
            : lambdaSquared{ lambdaSquared }
        {}

        [[nodiscard]] T DampingSquared(const math::Matrix<T, Residuals, Parameters>&) const override
        {
            return lambdaSquared;
        }

    private:
        T lambdaSquared;
    };
}
