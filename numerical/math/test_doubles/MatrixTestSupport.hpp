#pragma once

#include "numerical/math/Matrix.hpp"
#include "numerical/math/Tolerance.hpp"
#include <cmath>

namespace math::test
{
    template<typename T, std::size_t Rows, std::size_t Cols>
    bool AreMatricesNear(const math::Matrix<T, Rows, Cols>& a, const math::Matrix<T, Rows, Cols>& b, float eps = math::Tolerance<float>())
    {
        for (std::size_t i = 0; i < Rows; ++i)
            for (std::size_t j = 0; j < Cols; ++j)
                if (std::abs(math::ToFloat(a.at(i, j)) - math::ToFloat(b.at(i, j))) >= eps)
                    return false;
        return true;
    }

    template<typename T, std::size_t Size>
    bool AreVectorsNear(const math::Vector<T, Size>& a, const math::Vector<T, Size>& b, float eps = math::Tolerance<float>())
    {
        for (std::size_t i = 0; i < Size; ++i)
            if (std::abs(math::ToFloat(a.at(i, 0)) - math::ToFloat(b.at(i, 0))) >= eps)
                return false;
        return true;
    }
}
