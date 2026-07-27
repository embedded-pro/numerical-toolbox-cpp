// Copyright 2024 Numerical Toolbox Contributors. All rights reserved.
#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include <array>
#include <cstddef>
#include <type_traits>

namespace filters::passive
{
    namespace detail
    {
        template<typename T, std::size_t N>
        constexpr math::Matrix<T, N, N> InvertLu(math::Matrix<T, N, N> M)
        {
            math::Matrix<T, N, N> inv{};
            for (std::size_t i = 0; i < N; ++i)
                inv.at(i, i) = T{ 1 };

            for (std::size_t col = 0; col < N; ++col)
            {
                T pivot = M.at(col, col);
                for (std::size_t r = col + 1; r < N; ++r)
                {
                    T factor = M.at(r, col) / pivot;
                    for (std::size_t c = 0; c < N; ++c)
                    {
                        M.at(r, c) -= factor * M.at(col, c);
                        inv.at(r, c) -= factor * inv.at(col, c);
                    }
                }
            }

            for (std::size_t col2 = N; col2-- > 0;)
            {
                T pivot = M.at(col2, col2);
                for (std::size_t c = 0; c < N; ++c)
                    inv.at(col2, c) /= pivot;
                for (std::size_t r2 = col2; r2-- > 0;)
                {
                    T factor = M.at(r2, col2);
                    for (std::size_t c = 0; c < N; ++c)
                        inv.at(r2, c) -= factor * inv.at(col2, c);
                }
            }
            return inv;
        }

        template<typename T, std::size_t Window, std::size_t Order, std::size_t Deriv>
        constexpr std::array<T, Window> ComputeKernel()
        {
            constexpr std::size_t half = (Window - 1) / 2;

            math::Matrix<T, Window, Order + 1> A{};
            for (std::size_t row = 0; row < Window; ++row)
            {
                T offset = static_cast<T>(static_cast<int>(row) - static_cast<int>(half));
                T power = T{ 1 };
                for (std::size_t col = 0; col < Order + 1; ++col)
                {
                    A.at(row, col) = power;
                    power *= offset;
                }
            }

            math::Matrix<T, Order + 1, Order + 1> AtA{};
            for (std::size_t i = 0; i < Order + 1; ++i)
                for (std::size_t j = 0; j < Order + 1; ++j)
                    for (std::size_t k = 0; k < Window; ++k)
                        AtA.at(i, j) += A.at(k, i) * A.at(k, j);

            auto AtAinv = InvertLu<T, Order + 1>(AtA);

            math::Matrix<T, Order + 1, Window> C{};
            for (std::size_t i = 0; i < Order + 1; ++i)
                for (std::size_t j = 0; j < Window; ++j)
                    for (std::size_t k = 0; k < Order + 1; ++k)
                        C.at(i, j) += AtAinv.at(i, k) * A.at(j, k);

            std::array<T, Window> coeffs{};
            for (std::size_t j = 0; j < Window; ++j)
                coeffs[j] = C.at(Deriv, j);

            return coeffs;
        }
    }

    template<typename T, std::size_t Window, std::size_t Order, std::size_t Deriv = 0>
    class SavitzkyGolayFilter
    {
        static_assert(std::is_floating_point_v<T>, "SavitzkyGolayFilter supports floating-point types");
        static_assert(Window >= 3, "SavitzkyGolayFilter window must be at least 3");
        static_assert(Window % 2 == 1, "SavitzkyGolayFilter window must be odd");
        static_assert(Order < Window, "SavitzkyGolayFilter polynomial order must be less than window size");
        static_assert(Deriv <= Order, "SavitzkyGolayFilter derivative order must not exceed polynomial order");

    public:
        explicit SavitzkyGolayFilter(T initial = T{}) noexcept;

        OPTIMIZE_FOR_SPEED T Filter(T input) noexcept;
        void Reset(T value = T{}) noexcept;

        [[nodiscard]] static constexpr std::array<T, Window> Coefficients() noexcept;

    private:
        static constexpr std::array<T, Window> coeffs{ detail::ComputeKernel<T, Window, Order, Deriv>() };
        std::array<T, Window> line;
        std::size_t head;
    };

    template<typename T, std::size_t Window, std::size_t Order, std::size_t Deriv>
    SavitzkyGolayFilter<T, Window, Order, Deriv>::SavitzkyGolayFilter(T initial) noexcept
        : head{ 0 }
    {
        line.fill(initial);
    }

    template<typename T, std::size_t Window, std::size_t Order, std::size_t Deriv>
    OPTIMIZE_FOR_SPEED T SavitzkyGolayFilter<T, Window, Order, Deriv>::Filter(T input) noexcept
    {
        line[head] = input;
        head = (head + 1) % Window;

        T acc{};
        for (std::size_t k = 0; k < Window; ++k)
            acc += coeffs[k] * line[(head + k) % Window];

        return acc;
    }

    template<typename T, std::size_t Window, std::size_t Order, std::size_t Deriv>
    void SavitzkyGolayFilter<T, Window, Order, Deriv>::Reset(T value) noexcept
    {
        line.fill(value);
        head = 0;
    }

    template<typename T, std::size_t Window, std::size_t Order, std::size_t Deriv>
    constexpr std::array<T, Window> SavitzkyGolayFilter<T, Window, Order, Deriv>::Coefficients() noexcept
    {
        return coeffs;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class SavitzkyGolayFilter<float, 5, 2, 0>;
    extern template class SavitzkyGolayFilter<float, 5, 2, 1>;
#endif
}
