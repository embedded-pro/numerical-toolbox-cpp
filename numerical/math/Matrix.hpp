#ifndef MATH_MATRIX_HPP
#define MATH_MATRIX_HPP

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/QNumber.hpp"
#include <array>
#include <type_traits>

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

namespace math
{
    template<typename T, size_t Rows, size_t Cols>
    class Matrix;

    namespace detail
    {
        template<typename T>
        inline constexpr bool is_supported_type_v =
            std::disjunction_v<std::is_same<T, float>, is_qnumber<T>>;

        template<size_t Rows, size_t Cols>
        inline constexpr bool is_valid_dimensions_v = (Rows > 0 && Cols > 0);
    }

    template<typename T, size_t Rows, size_t Cols>
    class Matrix
    {
        static_assert(detail::is_supported_type_v<T>,
            "Matrix only supports float or QNumber types");
        static_assert(detail::is_valid_dimensions_v<Rows, Cols>,
            "Matrix dimensions must be positive");

    public:
        using value_type = T;
        using size_type = size_t;
        using reference = T&;
        using const_reference = const T&;
        using iterator = typename std::array<T, Rows * Cols>::iterator;
        using const_iterator = typename std::array<T, Rows * Cols>::const_iterator;

        static constexpr size_type rows = Rows;
        static constexpr size_type cols = Cols;
        static constexpr size_type size = Rows * Cols;

        constexpr Matrix() noexcept;
        template<typename... Args, typename = std::enable_if_t<sizeof...(Args) == Rows * Cols>>
        constexpr explicit Matrix(Args&&... args) noexcept;
        constexpr Matrix(std::initializer_list<std::initializer_list<T>> init);

        [[nodiscard]] constexpr reference at(size_type row, size_type col);
        [[nodiscard]] constexpr const_reference at(size_type row, size_type col) const;
        [[nodiscard]] constexpr reference operator[](size_type row);
        [[nodiscard]] constexpr const_reference operator[](size_type row) const;

        [[nodiscard]] constexpr iterator begin() noexcept;
        [[nodiscard]] constexpr iterator end() noexcept;
        [[nodiscard]] constexpr const_iterator begin() const noexcept;
        [[nodiscard]] constexpr const_iterator end() const noexcept;

        [[nodiscard]] constexpr Matrix operator+(const Matrix& rhs) const;
        [[nodiscard]] constexpr Matrix operator-(const Matrix& rhs) const;
        template<size_t RhsCols>
        [[nodiscard]] constexpr Matrix<T, Rows, RhsCols> operator*(const Matrix<T, Cols, RhsCols>& rhs) const;
        [[nodiscard]] constexpr Matrix operator*(const T& scalar) const;

        constexpr Matrix& operator+=(const Matrix& rhs);
        constexpr Matrix& operator-=(const Matrix& rhs);
        constexpr Matrix& operator*=(const T& scalar);

        [[nodiscard]] constexpr Matrix<T, Cols, Rows> Transpose() const;
        [[nodiscard]] static constexpr Matrix Identity();
        [[nodiscard]] constexpr T Trace() const;

    private:
        std::array<T, Rows * Cols> data;
    };

    template<typename T, typename... U>
    Matrix(T, U...) -> Matrix<T, 1, 1 + sizeof...(U)>;

    template<typename T, size_t Size>
    using SquareMatrix = Matrix<T, Size, Size>;

    template<typename T, size_t Size>
    using Vector = Matrix<T, Size, 1>;

    template<typename T, size_t Rows, size_t Cols>
    [[nodiscard]] constexpr Matrix<T, Rows, Cols>
    MakeMatrix(std::initializer_list<std::initializer_list<T>> init)
    {
        return Matrix<T, Rows, Cols>(init);
    }

    // Implementation //

    template<typename T, size_t Rows, size_t Cols>
    constexpr Matrix<T, Rows, Cols>::Matrix() noexcept
        : data{}
    {}

    template<typename T, size_t Rows, size_t Cols>
    template<typename... Args, typename>
    OPTIMIZE_FOR_SPEED constexpr Matrix<T, Rows, Cols>::Matrix(Args&&... args) noexcept
        : data{ std::forward<Args>(args)... }
    {}

    template<typename T, size_t Rows, size_t Cols>
    OPTIMIZE_FOR_SPEED constexpr Matrix<T, Rows, Cols>::Matrix(std::initializer_list<std::initializer_list<T>> init)
    {
        size_t row = 0;
        for (const auto& row_list : init)
        {
            size_t col = 0;
            for (const auto& value : row_list)
            {
                at(row, col) = value;
                ++col;
            }
            ++row;
        }
    }

    template<typename T, size_t Rows, size_t Cols>
    OPTIMIZE_FOR_SPEED constexpr typename Matrix<T, Rows, Cols>::reference
    Matrix<T, Rows, Cols>::at(size_type row, size_type col)
    {
#ifdef NUMERICAL_TOOLBOX_ENABLE_ASSERTIONS
        really_assert(row < Rows && col < Cols);
#endif
        return data[row * Cols + col];
    }

    template<typename T, size_t Rows, size_t Cols>
    OPTIMIZE_FOR_SPEED constexpr typename Matrix<T, Rows, Cols>::const_reference
    Matrix<T, Rows, Cols>::at(size_type row, size_type col) const
    {
#ifdef NUMERICAL_TOOLBOX_ENABLE_ASSERTIONS
        really_assert(row < Rows && col < Cols);
#endif
        return data[row * Cols + col];
    }

    template<typename T, size_t Rows, size_t Cols>
    OPTIMIZE_FOR_SPEED constexpr typename Matrix<T, Rows, Cols>::reference
    Matrix<T, Rows, Cols>::operator[](size_type row)
    {
#ifdef NUMERICAL_TOOLBOX_ENABLE_ASSERTIONS
        really_assert(row < Rows);
#endif
        return data[row * Cols];
    }

    template<typename T, size_t Rows, size_t Cols>
    OPTIMIZE_FOR_SPEED constexpr typename Matrix<T, Rows, Cols>::const_reference
    Matrix<T, Rows, Cols>::operator[](size_type row) const
    {
#ifdef NUMERICAL_TOOLBOX_ENABLE_ASSERTIONS
        really_assert(row < Rows);
#endif
        return data[row * Cols];
    }

    template<typename T, size_t Rows, size_t Cols>
    OPTIMIZE_FOR_SPEED constexpr typename Matrix<T, Rows, Cols>::iterator
    Matrix<T, Rows, Cols>::begin() noexcept
    {
        return data.begin();
    }

    template<typename T, size_t Rows, size_t Cols>
    OPTIMIZE_FOR_SPEED constexpr typename Matrix<T, Rows, Cols>::iterator
    Matrix<T, Rows, Cols>::end() noexcept
    {
        return data.end();
    }

    template<typename T, size_t Rows, size_t Cols>
    OPTIMIZE_FOR_SPEED constexpr typename Matrix<T, Rows, Cols>::const_iterator
    Matrix<T, Rows, Cols>::begin() const noexcept
    {
        return data.begin();
    }

    template<typename T, size_t Rows, size_t Cols>
    OPTIMIZE_FOR_SPEED constexpr typename Matrix<T, Rows, Cols>::const_iterator
    Matrix<T, Rows, Cols>::end() const noexcept
    {
        return data.end();
    }

    template<typename T, size_t Rows, size_t Cols>
    OPTIMIZE_FOR_SPEED constexpr Matrix<T, Rows, Cols>
    Matrix<T, Rows, Cols>::operator+(const Matrix& rhs) const
    {
        Matrix result;
        for (size_type i = 0; i < size; ++i)
            result.data[i] = data[i] + rhs.data[i];

        return result;
    }

    template<typename T, size_t Rows, size_t Cols>
    OPTIMIZE_FOR_SPEED constexpr Matrix<T, Rows, Cols>
    Matrix<T, Rows, Cols>::operator-(const Matrix& rhs) const
    {
        Matrix result;
        for (size_type i = 0; i < size; ++i)
            result.data[i] = data[i] - rhs.data[i];

        return result;
    }

    template<typename T, size_t Rows, size_t Cols>
    template<size_t RhsCols>
    OPTIMIZE_FOR_SPEED constexpr Matrix<T, Rows, RhsCols>
    Matrix<T, Rows, Cols>::operator*(const Matrix<T, Cols, RhsCols>& rhs) const
    {
        Matrix<T, Rows, RhsCols> result;
        for (size_type i = 0; i < Rows; ++i)
        {
            for (size_type j = 0; j < RhsCols; ++j)
            {
                T sum{};
                for (size_type k = 0; k < Cols; ++k)
                    sum += at(i, k) * rhs.at(k, j);

                result.at(i, j) = sum;
            }
        }
        return result;
    }

    template<typename T, size_t Rows, size_t Cols>
    OPTIMIZE_FOR_SPEED constexpr Matrix<T, Rows, Cols>
    Matrix<T, Rows, Cols>::operator*(const T& scalar) const
    {
        Matrix result;
        for (size_type i = 0; i < size; ++i)
            result.data[i] = data[i] * scalar;

        return result;
    }

    template<typename T, size_t Rows, size_t Cols>
    OPTIMIZE_FOR_SPEED constexpr Matrix<T, Rows, Cols>&
    Matrix<T, Rows, Cols>::operator+=(const Matrix& rhs)
    {
        for (size_type i = 0; i < size; ++i)
            data[i] += rhs.data[i];

        return *this;
    }

    template<typename T, size_t Rows, size_t Cols>
    OPTIMIZE_FOR_SPEED constexpr Matrix<T, Rows, Cols>&
    Matrix<T, Rows, Cols>::operator-=(const Matrix& rhs)
    {
        for (size_type i = 0; i < size; ++i)
            data[i] -= rhs.data[i];

        return *this;
    }

    template<typename T, size_t Rows, size_t Cols>
    OPTIMIZE_FOR_SPEED constexpr Matrix<T, Rows, Cols>&
    Matrix<T, Rows, Cols>::operator*=(const T& scalar)
    {
        for (size_type i = 0; i < size; ++i)
            data[i] *= scalar;

        return *this;
    }

    template<typename T, size_t Rows, size_t Cols>
    OPTIMIZE_FOR_SPEED constexpr Matrix<T, Cols, Rows>
    Matrix<T, Rows, Cols>::Transpose() const
    {
        Matrix<T, Cols, Rows> result;
        for (size_type i = 0; i < Rows; ++i)
            for (size_type j = 0; j < Cols; ++j)
                result.at(j, i) = at(i, j);

        return result;
    }

    template<typename T, size_t Rows, size_t Cols>
    OPTIMIZE_FOR_SPEED constexpr Matrix<T, Rows, Cols>
    Matrix<T, Rows, Cols>::Identity()
    {
        static_assert(Rows == Cols,
            "Identity matrix can only be created for square matrices");

        Matrix result;
        for (size_type i = 0; i < Rows; ++i)
            result.at(i, i) = T(0.9999f);

        return result;
    }

    template<typename T, size_t Rows, size_t Cols>
    OPTIMIZE_FOR_SPEED constexpr T
    Matrix<T, Rows, Cols>::Trace() const
    {
        static_assert(Rows == Cols,
            "Trace can only be computed for square matrices");

        T sum{};
        for (size_type i = 0; i < Rows; ++i)
            sum += at(i, i);

        return sum;
    }
}

#endif
