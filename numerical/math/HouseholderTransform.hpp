#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/Math.hpp"
#include <cstddef>
#include <type_traits>

namespace math
{
    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED void HouseholderVector(const Vector<T, N>& x, std::size_t start, Vector<T, N>& v, T& beta)
    {
        static_assert(std::is_floating_point_v<T>, "HouseholderVector supports floating-point types only");

        T sigma{ T{} };
        for (std::size_t i = start + 1; i < N; ++i)
            sigma += x.at(i, 0) * x.at(i, 0);

        for (std::size_t i = 0; i < N; ++i)
            v.at(i, 0) = (i >= start) ? x.at(i, 0) : T{};

        if (sigma < T{ 1e-30f })
        {
            beta = T{};
            return;
        }

        T norm = math::Sqrt(x.at(start, 0) * x.at(start, 0) + sigma);
        T v0;
        if (x.at(start, 0) <= T{})
            v0 = x.at(start, 0) - norm;
        else
            v0 = -sigma / (x.at(start, 0) + norm);

        beta = T{ 2 } * v0 * v0 / (sigma + v0 * v0);

        T invV0 = T{ 1 } / v0;
        v.at(start, 0) = T{ 1 };
        for (std::size_t i = start + 1; i < N; ++i)
            v.at(i, 0) = x.at(i, 0) * invV0;
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    OPTIMIZE_FOR_SPEED void ApplyReflectorLeft(Matrix<T, Rows, Cols>& a, const Vector<T, Rows>& v, T beta,
        std::size_t rowStart, std::size_t colStart)
    {
        static_assert(std::is_floating_point_v<T>, "ApplyReflectorLeft supports floating-point types only");

        for (std::size_t j = colStart; j < Cols; ++j)
        {
            T dot{ T{} };
            for (std::size_t i = rowStart; i < Rows; ++i)
                dot += v.at(i, 0) * a.at(i, j);

            T scale = beta * dot;
            for (std::size_t i = rowStart; i < Rows; ++i)
                a.at(i, j) -= scale * v.at(i, 0);
        }
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    OPTIMIZE_FOR_SPEED void ApplyReflectorRight(Matrix<T, Rows, Cols>& a, const Vector<T, Cols>& v, T beta,
        std::size_t rowStart, std::size_t colStart)
    {
        static_assert(std::is_floating_point_v<T>, "ApplyReflectorRight supports floating-point types only");

        for (std::size_t i = rowStart; i < Rows; ++i)
        {
            T dot{ T{} };
            for (std::size_t j = colStart; j < Cols; ++j)
                dot += a.at(i, j) * v.at(j, 0);

            T scale = beta * dot;
            for (std::size_t j = colStart; j < Cols; ++j)
                a.at(i, j) -= scale * v.at(j, 0);
        }
    }
}
