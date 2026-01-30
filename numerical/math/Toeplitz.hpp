#ifndef MATH_TOEPLITZ_MATRIX_HPP
#define MATH_TOEPLITZ_MATRIX_HPP

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

namespace math
{
    template<typename T, std::size_t N>
    class ToeplitzMatrix
    {
        static_assert(detail::is_supported_type_v<T>,
            "ToeplitzMatrix only supports float or QNumber types");
        static_assert(detail::is_valid_dimensions_v<N, N>,
            "ToeplitzMatrix dimensions must be positive");

    public:
        using value_type = T;
        using size_type = std::size_t;
        using reference = T&;
        using const_reference = const T&;

        static constexpr size_type size = N;
        static constexpr T epsilon = T(1e-6f);

        constexpr ToeplitzMatrix() noexcept;
        constexpr ToeplitzMatrix(const Vector<T, N>& first_row, const Vector<T, N>& first_col);
        explicit constexpr ToeplitzMatrix(const Vector<T, N>& autocorrelation);

        [[nodiscard]] constexpr T at(size_type row, size_type col) const noexcept;
        [[nodiscard]] constexpr Matrix<T, N, N> ToFullMatrix() const;

        [[nodiscard]] constexpr Vector<T, N> operator*(const Vector<T, N>& vec) const noexcept;
        [[nodiscard]] constexpr ToeplitzMatrix operator+(const ToeplitzMatrix& other) const;
        [[nodiscard]] constexpr ToeplitzMatrix operator-(const ToeplitzMatrix& other) const;

        [[nodiscard]] constexpr bool IsSymmetric() const;

        [[nodiscard]] static constexpr bool IsToeplitzMatrix(const Matrix<T, N, N>& matrix);
        [[nodiscard]] static constexpr std::pair<Vector<T, N>, Vector<T, N>> ExtractToeplitzVectors(const Matrix<T, N, N>& matrix);

    private:
        Vector<T, N> first_row;
        Vector<T, N> first_col;
    };

    // Implementation //

    template<typename T, std::size_t N>
    constexpr ToeplitzMatrix<T, N>::ToeplitzMatrix() noexcept
        : first_row()
        , first_col()
    {}

    template<typename T, std::size_t N>
    constexpr ToeplitzMatrix<T, N>::ToeplitzMatrix(const Vector<T, N>& first_row, const Vector<T, N>& first_col)
        : first_row(first_row)
        , first_col(first_col)
    {
#ifndef NDEBUG
        // First element must be the same in row and column (debug check only)
        really_assert(first_row.at(0, 0) == first_col.at(0, 0));
#endif
    }

    template<typename T, std::size_t N>
    constexpr ToeplitzMatrix<T, N>::ToeplitzMatrix(const Vector<T, N>& autocorrelation)
        : first_row(autocorrelation)
        , first_col(autocorrelation)
    {}

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED constexpr T ToeplitzMatrix<T, N>::at(size_type row, size_type col) const noexcept
    {
        if (row <= col) [[likely]]
            return first_row.at(col - row, 0);
        else
            return first_col.at(row - col, 0);
    }

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED constexpr Matrix<T, N, N> ToeplitzMatrix<T, N>::ToFullMatrix() const
    {
        Matrix<T, N, N> result;
        for (size_type i = 0; i < N; ++i)
            for (size_type j = 0; j < N; ++j)
                result.at(i, j) = at(i, j);

        return result;
    }

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED constexpr Vector<T, N> ToeplitzMatrix<T, N>::operator*(const Vector<T, N>& vec) const noexcept
    {
        Vector<T, N> result;

        for (size_type i = 0; i < N; ++i)
        {
            T sum{};

            for (size_type j = i; j < N; ++j)
                sum += first_row.at(j - i, 0) * vec.at(j, 0);

            for (size_type j = i; j-- > 0;)
                sum += first_col.at(i - j, 0) * vec.at(j, 0);

            result.at(i, 0) = sum;
        }

        return result;
    }

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED constexpr ToeplitzMatrix<T, N> ToeplitzMatrix<T, N>::operator+(const ToeplitzMatrix& other) const
    {
        return ToeplitzMatrix(first_row + other.first_row, first_col + other.first_col);
    }

    template<typename T, std::size_t N>
    OPTIMIZE_FOR_SPEED constexpr ToeplitzMatrix<T, N> ToeplitzMatrix<T, N>::operator-(const ToeplitzMatrix& other) const
    {
        return ToeplitzMatrix(first_row - other.first_row, first_col - other.first_col);
    }

    template<typename T, std::size_t N>
    constexpr bool ToeplitzMatrix<T, N>::IsSymmetric() const
    {
        for (size_type i = 0; i < N; ++i)
        {
            if (!(first_row.at(i, 0) == first_col.at(i, 0)))
                return false;
        }

        return true;
    }

    template<typename T, std::size_t N>
    constexpr bool ToeplitzMatrix<T, N>::IsToeplitzMatrix(const Matrix<T, N, N>& matrix)
    {
        for (size_t i = 1; i < N; ++i)
            for (size_t j = 1; j < N; ++j)
                if (std::abs(math::ToFloat(matrix.at(i, j)) - math::ToFloat(matrix.at(i - 1, j - 1))) > math::ToFloat(epsilon))
                    return false;

        return true;
    }

    template<typename T, std::size_t N>
    constexpr std::pair<Vector<T, N>, Vector<T, N>>
    ToeplitzMatrix<T, N>::ExtractToeplitzVectors(const Matrix<T, N, N>& matrix)
    {
        Vector<T, N> first_row;
        Vector<T, N> first_col;

        for (size_t i = 0; i < N; ++i)
        {
            first_row.at(i, 0) = matrix.at(0, i);
            first_col.at(i, 0) = matrix.at(i, 0);
        }

        return { first_row, first_col };
    }

    template<typename T, std::size_t N>
    [[nodiscard]] constexpr ToeplitzMatrix<T, N> CreateToeplitzMatrix(const Vector<T, N>& autocorrelation)
    {
        return ToeplitzMatrix<T, N>(autocorrelation);
    }
}

#endif
