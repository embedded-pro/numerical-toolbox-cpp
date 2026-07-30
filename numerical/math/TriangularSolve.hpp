#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/QNumber.hpp"
#include <cmath>
#include <cstddef>

namespace math
{
    template<typename T, std::size_t N>
    [[nodiscard]] OPTIMIZE_FOR_SPEED Vector<T, N> SolveUpperTriangular(const Matrix<T, N, N>& r, const Vector<T, N>& c)
    {
        Vector<T, N> x{};

        for (std::size_t i = N; i > 0; --i)
        {
            std::size_t row = i - 1;
            T sum = c.at(row, 0);
            for (std::size_t j = row + 1; j < N; ++j)
                sum = sum - r.at(row, j) * x.at(j, 0);

            really_assert(std::abs(ToFloat(r.at(row, row))) > 0.0f);
            x.at(row, 0) = sum / r.at(row, row);
        }

        return x;
    }
}
