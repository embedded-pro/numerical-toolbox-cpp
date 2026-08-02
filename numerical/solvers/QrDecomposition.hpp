#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/GivensRotation.hpp"
#include "numerical/math/HouseholderTransform.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/TriangularSolve.hpp"
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
    };

    template<typename T, std::size_t Rows, std::size_t Cols>
    OPTIMIZE_FOR_SPEED bool QrDecomposition<T, Rows, Cols>::Decompose(const math::Matrix<T, Rows, Cols>& a)
    {
        qr = a;
        betas = math::Vector<T, Cols>{};
        factored = false;

        math::Vector<T, Rows> colVec{};
        math::Vector<T, Rows> v{};

        T maxPivot{ T{} };

        for (std::size_t k = 0; k < Cols; ++k)
        {
            for (std::size_t i = 0; i < Rows; ++i)
                colVec.at(i, 0) = (i >= k) ? qr.at(i, k) : T{};

            T beta{};
            math::HouseholderVector(colVec, k, v, beta);
            betas.at(k, 0) = beta;

            if (beta != T{})
            {
                math::ApplyReflectorLeft(qr, v, beta, k, k);

                for (std::size_t i = k + 1; i < Rows; ++i)
                    qr.at(i, k) = v.at(i, 0);
            }

            T pivot = std::abs(qr.at(k, k));
            if (pivot > maxPivot)
                maxPivot = pivot;

            if (pivot <= maxPivot * T{ 1e-6f })
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

            math::ApplyReflectorLeft(result, v, beta, col, col);
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
    OPTIMIZE_FOR_SPEED math::Vector<T, Cols>
    QrDecomposition<T, Rows, Cols>::SolveLeastSquares(const math::Vector<T, Rows>& b) const
    {
        math::Vector<T, Rows> c = b;
        ApplyQtranspose(c);

        math::Vector<T, Cols> c1{};
        for (std::size_t i = 0; i < Cols; ++i)
            c1.at(i, 0) = c.at(i, 0);

        return math::SolveUpperTriangular(R(), c1);
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    void QrDecomposition<T, Rows, Cols>::GivensUpdateRow(const math::Matrix<T, 1, Cols>& newRow)
    {
        math::Matrix<T, 1, Cols> row = newRow;

        for (std::size_t k = 0; k < Cols; ++k)
        {
            math::GivensRotation<T> g = math::ComputeGivens(qr.at(k, k), row.at(0, k));

            for (std::size_t j = k; j < Cols; ++j)
                math::ApplyGivens(g, qr.at(k, j), row.at(0, j));
        }
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class QrDecomposition<float, 4, 3>;
    extern template class QrDecomposition<float, 3, 3>;
#endif
}
