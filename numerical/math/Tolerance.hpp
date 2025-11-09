#pragma once

#include "numerical/math/QNumber.hpp"

namespace math
{
    template<typename T>
    float Tolerance()
    {
        if constexpr (std::is_same_v<T, float>)
            return 1e-3f;
        else if constexpr (std::is_same_v<T, math::Q31>)
            return 1e-3f;
        else
            return 1e-3f;
    }
}
