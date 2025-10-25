#pragma once

#include "numerical/math/QNumber.hpp"

namespace controllers
{
    template<typename T>
    float GetTolerance()
    {
        if constexpr (std::is_same_v<T, float>)
            return 1e-3f;
        else if constexpr (std::is_same_v<T, math::Q31>)
            return 5e-2f;
        else
            return 7e-2f;
    }
}
