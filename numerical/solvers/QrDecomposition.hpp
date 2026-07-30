#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include <cmath>
#include <cstddef>
#include <type_traits>

namespace solvers
{
    template<typename T, std::size_t Rows, std::size_t Cols>
    class QrDecomposition
    {
        static_assert(std::is_floating_point_v<T>, "QrDecomposition supports floating-point types only");
        static_assert(Rows >= Cols, "QrDecomposition requires Rows >= Cols");

    public:
        QrDecomposition() = default;

        OPTIMIZE_FOR_SPEED bool Decompose(const math::Matrix<T, Rows, Cols>& a);
        math::Matrix<T, Rows, Cols> Q() const;
        math::Matrix<T, Cols, Cols> R() const;
        OPTIMIZE_FOR_SPEED math::Vector<T, Cols> SolveLeastSquares(const math::Vector<T, Rows>& b) const;
        void ApplyQtranspose(math::Vector<T, Rows>& v) const;
        void GivensUpdateRow(const math::Matrix<T, 1, Cols>& newRow);

    private:
        math::Matrix<T, Rows, Cols> qr{};
        math::Vector<T, Cols> betas{};
        bool factored{ false };

        static void HouseholderVector(const math::Vector<T, Rows>& x, std::size_t start, math::Vector<T, Rows>& v, T& beta);
        math::Vector<T, Cols> BackSubstitute(const math::Matrix<T, Cols, Cols>& r, const math::Vector<T, Cols>& c) const;
    };

    template<typename T, std::size_t Rows, std::size_t Cols>
    void QrDecomposition<T, Rows, Cols>::HouseholderVector(
        const math::Vector<T, Rows>& x, std::size_t start, math::Vector<T, Rows>& v, T& beta)
    {
        T sigma{ T{} };
        for (std::size_t i = start + 1; i < Rows; ++i)
            sigma += x.at(i, 0) * x.at(i, 0);

        for (std::size_t i = 0; i < Rows; ++i)
            v.at(i, 0) = (i >= start) ? x.at(i, 0) : T{};

        if (sigma < T{ 1e-30f })
        {
            beta = T{};
            return;
        }

        T norm = std::sqrt(x.at(start, 0) * x.at(start, 0) + sigma);
        T v0;
        if (x.at(start, 0) <= T{})
            v0 = x.at(start, 0) - norm;
        else
            v0 = -sigma / (x.at(start, 0) + norm);

        beta = T{ 2 } * v0 * v0 / (sigma + v0 * v0);

        T invV0 = T{ 1 } / v0;
        v.at(start, 0) = T{ 1 };
        for (std::size_t i = start + 1; i < Rows; ++i)
            v.at(i, 0) = x.at(i, 0) * invV0;
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    OPTIMIZE_FOR_SPEED bool QrDecomposition<T, Rows, Cols>::Decompose(const math::Matrix<T, Rows, Cols>& a)
    {
        qr = a;
        betas = math::Vector<T, Cols>{};
        factored = false;

        math::Vector<T, Rows> colVec{};
        math::Vector<T, Rows> v{};

        for (std::size_t k = 0; k < Cols; ++k)
        {
            for (std::size_t i = 0; i < Rows; ++i)
                colVec.at(i, 0) = (i >= k) ? qr.at(i, k) : T{};

            T beta{};
            HouseholderVector(colVec, k, v, beta);
            betas.at(k, 0) = beta;

            if (beta != T{})
            {
                for (std::size_t j = k; j < Cols; ++j)
                {
                    T dot{ T{} };
                    for (std::size_t i = k; i < Rows; ++i)
                        dot += v.at(i, 0) * qr.at(i, j);

                    T scale = beta * dot;
                    for (std::size_t i = k; i < Rows; ++i)
                        qr.at(i, j) -= scale * v.at(i, 0);
                }

                for (std::size_t i = k + 1; i < Rows; ++i)
                    qr.at(i, k) = v.at(i, 0);
            }

            if (std::abs(qr.at(k, k)) < T{ 1e-12f })
                return false;
        }

        factored = true;
        return true;
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    math::Matrix<T, Rows, Cols> QrDecomposition<T, Rows, Cols>::Q() const
    {
        math::Matrix<T, Rows, Cols> result{};
        for (std::size_t j = 0; j < Cols; ++j)
            result.at(j, j) = T{ 1 };

        math::Vector<T, Rows> v{};

        for (std::size_t k = Cols; k > 0; --k)
        {
            std::size_t col = k - 1;
            T beta = betas.at(col, 0);
            if (beta == T{})
                continue;

            v.at(col, 0) = T{ 1 };
            for (std::size_t i = col + 1; i < Rows; ++i)
                v.at(i, 0) = qr.at(i, col);

            for (std::size_t j = col; j < Cols; ++j)
            {
                T dot{ T{} };
                for (std::size_t i = col; i < Rows; ++i)
                    dot += v.at(i, 0) * result.at(i, j);

                T scale = beta * dot;
                for (std::size_t i = col; i < Rows; ++i)
                    result.at(i, j) -= scale * v.at(i, 0);
            }
        }

        return result;
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    math::Matrix<T, Cols, Cols> QrDecomposition<T, Rows, Cols>::R() const
    {
        math::Matrix<T, Cols, Cols> r{};
        for (std::size_t i = 0; i < Cols; ++i)
            for (std::size_t j = i; j < Cols; ++j)
                r.at(i, j) = qr.at(i, j);
        return r;
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    void QrDecomposition<T, Rows, Cols>::ApplyQtranspose(math::Vector<T, Rows>& v) const
    {
        math::Vector<T, Rows> hv{};

        for (std::size_t k = 0; k < Cols; ++k)
        {
            T beta = betas.at(k, 0);
            if (beta == T{})
                continue;

            hv.at(k, 0) = T{ 1 };
            for (std::size_t i = k + 1; i < Rows; ++i)
                hv.at(i, 0) = qr.at(i, k);

            T dot{ T{} };
            for (std::size_t i = k; i < Rows; ++i)
                dot += hv.at(i, 0) * v.at(i, 0);

            T scale = beta * dot;
            for (std::size_t i = k; i < Rows; ++i)
                v.at(i, 0) -= scale * hv.at(i, 0);
        }
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    math::Vector<T, Cols> QrDecomposition<T, Rows, Cols>::BackSubstitute(
        const math::Matrix<T, Cols, Cols>& r, const math::Vector<T, Cols>& c) const
    {
        math::Vector<T, Cols> x{};

        for (std::size_t i = Cols; i > 0; --i)
        {
            std::size_t row = i - 1;
            T sum = c.at(row, 0);
            for (std::size_t j = row + 1; j < Cols; ++j)
                sum -= r.at(row, j) * x.at(j, 0);
            x.at(row, 0) = sum / r.at(row, row);
        }

        return x;
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    OPTIMIZE_FOR_SPEED math::Vector<T, Cols>
    QrDecomposition<T, Rows, Cols>::SolveLeastSquares(const math::Vector<T, Rows>& b) const
    {
        math::Vector<T, Rows> c = b;
        ApplyQtranspose(c);

        math::Vector<T, Cols> c1{};
        for (std::size_t i = 0; i < Cols; ++i)
            c1.at(i, 0) = c.at(i, 0);

        return BackSubstitute(R(), c1);
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    void QrDecomposition<T, Rows, Cols>::GivensUpdateRow(const math::Matrix<T, 1, Cols>& newRow)
    {
        math::Matrix<T, 1, Cols> row = newRow;

        for (std::size_t k = 0; k < Cols; ++k)
        {
            T rkk = qr.at(k, k);
            T xk = row.at(0, k);

            T rr = std::sqrt(rkk * rkk + xk * xk);
            if (rr < T{ 1e-30f })
                continue;

            T c = rkk / rr;
            T s = xk / rr;

            for (std::size_t j = k; j < Cols; ++j)
            {
                T rval = qr.at(k, j);
                T xval = row.at(0, j);
                qr.at(k, j) = c * rval + s * xval;
                row.at(0, j) = -s * rval + c * xval;
            }
        }
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class QrDecomposition<float, 4, 3>;
    extern template class QrDecomposition<float, 3, 3>;
#endif
}
