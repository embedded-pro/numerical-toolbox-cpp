#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

namespace math
{
    template<typename T>
    float Tolerance()
    {
        return 1e-3f;
    }
}
