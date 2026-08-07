#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/solvers/DurandKerner.hpp"
#include <array>
#include <cstddef>
#include <span>
#include <type_traits>

namespace solvers
{
    template<typename T, std::size_t Size>
    class SpectralRadius
    {
        static_assert(std::is_floating_point_v<T>,
            "SpectralRadius supports floating-point types only");

    public:
        OPTIMIZE_FOR_SPEED T Compute(const math::SquareMatrix<T, Size>& matrix) const;
        OPTIMIZE_FOR_SPEED bool IsSchurStable(const math::SquareMatrix<T, Size>& matrix) const;
        OPTIMIZE_FOR_SPEED T StabilityMargin(const math::SquareMatrix<T, Size>& matrix) const;

    private:
        static OPTIMIZE_FOR_SPEED math::SquareMatrix<T, Size> MakeIdentity();
        static OPTIMIZE_FOR_SPEED std::array<T, Size + 1> CharacteristicPolynomial(const math::SquareMatrix<T, Size>& matrix);
    };

    template<typename T, std::size_t Size>
    math::SquareMatrix<T, Size> SpectralRadius<T, Size>::MakeIdentity()
    {
        math::SquareMatrix<T, Size> result{};
        for (std::size_t i = 0; i < Size; ++i)
            result.at(i, i) = T(1);
        return result;
    }

    template<typename T, std::size_t Size>
    std::array<T, Size + 1> SpectralRadius<T, Size>::CharacteristicPolynomial(const math::SquareMatrix<T, Size>& matrix)
    {
        std::array<T, Size + 1> coefficients{};
        coefficients[0] = T(1);

        math::SquareMatrix<T, Size> M{};
        const math::SquareMatrix<T, Size> identity = MakeIdentity();

        for (std::size_t k = 1; k <= Size; ++k)
        {
            M = matrix * M + identity * coefficients[k - 1];
            coefficients[k] = -(matrix * M).Trace() / static_cast<T>(k);
        }

        return coefficients;
    }

    template<typename T, std::size_t Size>
    T SpectralRadius<T, Size>::Compute(const math::SquareMatrix<T, Size>& matrix) const
    {
        if constexpr (Size == 1)
            return math::Abs(matrix.at(0, 0));

        const std::array<T, Size + 1> poly = CharacteristicPolynomial(matrix);

        DurandKerner<T, Size> solver{};
        auto roots = solver.Solve(std::span<const T>{ poly.data(), poly.size() });

        T rho{};
        for (std::size_t i = 0; i < roots.size(); ++i)
        {
            T magnitude = math::Abs(roots[i]);
            if (magnitude > rho)
                rho = magnitude;
        }

        return rho;
    }

    template<typename T, std::size_t Size>
    bool SpectralRadius<T, Size>::IsSchurStable(const math::SquareMatrix<T, Size>& matrix) const
    {
        return Compute(matrix) < T(1);
    }

    template<typename T, std::size_t Size>
    T SpectralRadius<T, Size>::StabilityMargin(const math::SquareMatrix<T, Size>& matrix) const
    {
        return T(1) - Compute(matrix);
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class SpectralRadius<float, 2>;
    extern template class SpectralRadius<float, 3>;
#endif
}
