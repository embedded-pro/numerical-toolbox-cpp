#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/GivensRotation.hpp"
#include "numerical/math/HouseholderTransform.hpp"
#include "numerical/math/Matrix.hpp"
#include <array>
#include <cmath>
#include <cstddef>
#include <type_traits>

namespace solvers
{
    template<typename T, std::size_t Rows, std::size_t Cols>
    class SingularValueDecomposition
    {
        static_assert(std::is_floating_point_v<T>, "SingularValueDecomposition supports floating-point types only");
        static_assert(Rows >= Cols, "SingularValueDecomposition requires Rows >= Cols");

    public:
        SingularValueDecomposition() = default;

        OPTIMIZE_FOR_SPEED bool Decompose(const math::Matrix<T, Rows, Cols>& a);
        const math::Vector<T, Cols>& SingularValues() const;
        math::Matrix<T, Cols, Rows> PseudoInverse(T tol) const;
        std::size_t Rank(T tol) const;
        T ConditionNumber() const;
        math::Vector<T, Cols> SolveLeastSquares(const math::Vector<T, Rows>& b) const;

        const math::Matrix<T, Rows, Cols>& U() const;
        const math::Matrix<T, Cols, Cols>& V() const;

    private:
        static void ApplyLeftReflector(math::Matrix<T, Rows, Cols>& b,
            const math::Vector<T, Rows>& v, T beta, std::size_t col);
        static void ApplyRightReflector(math::Matrix<T, Rows, Cols>& b,
            const math::Vector<T, Cols>& v, T beta, std::size_t startCol);
        static void AccumulateLeft(math::Matrix<T, Rows, Cols>& u,
            const math::Vector<T, Rows>& v, T beta, std::size_t start);
        static void AccumulateRight(math::Matrix<T, Cols, Cols>& vMat,
            const math::Vector<T, Cols>& v, T beta, std::size_t start);
        void QrSweep(std::size_t p, std::size_t q);

        math::Matrix<T, Rows, Cols> uMat{};
        math::Vector<T, Cols> sigma{};
        math::Vector<T, Cols> superdiag{};
        math::Matrix<T, Cols, Cols> vMat{};
    };

    template<typename T, std::size_t Rows, std::size_t Cols>
    void SingularValueDecomposition<T, Rows, Cols>::ApplyLeftReflector(
        math::Matrix<T, Rows, Cols>& b,
        const math::Vector<T, Rows>& v, T beta, std::size_t col)
    {
        for (std::size_t j = col; j < Cols; ++j)
        {
            T dot{ T{} };
            for (std::size_t i = col; i < Rows; ++i)
                dot += v.at(i, 0) * b.at(i, j);
            T scale = beta * dot;
            for (std::size_t i = col; i < Rows; ++i)
                b.at(i, j) -= scale * v.at(i, 0);
        }
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    void SingularValueDecomposition<T, Rows, Cols>::ApplyRightReflector(
        math::Matrix<T, Rows, Cols>& b,
        const math::Vector<T, Cols>& v, T beta, std::size_t startCol)
    {
        for (std::size_t i = 0; i < Rows; ++i)
        {
            T dot{ T{} };
            for (std::size_t j = startCol; j < Cols; ++j)
                dot += b.at(i, j) * v.at(j, 0);
            T scale = beta * dot;
            for (std::size_t j = startCol; j < Cols; ++j)
                b.at(i, j) -= scale * v.at(j, 0);
        }
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    void SingularValueDecomposition<T, Rows, Cols>::AccumulateLeft(
        math::Matrix<T, Rows, Cols>& u,
        const math::Vector<T, Rows>& v, T beta, std::size_t start)
    {
        for (std::size_t j = 0; j < Cols; ++j)
        {
            T dot{ T{} };
            for (std::size_t i = start; i < Rows; ++i)
                dot += v.at(i, 0) * u.at(i, j);
            T scale = beta * dot;
            for (std::size_t i = start; i < Rows; ++i)
                u.at(i, j) -= scale * v.at(i, 0);
        }
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    void SingularValueDecomposition<T, Rows, Cols>::AccumulateRight(
        math::Matrix<T, Cols, Cols>& vMatrix,
        const math::Vector<T, Cols>& v, T beta, std::size_t start)
    {
        for (std::size_t i = 0; i < Cols; ++i)
        {
            T dot{ T{} };
            for (std::size_t j = start; j < Cols; ++j)
                dot += vMatrix.at(i, j) * v.at(j, 0);
            T scale = beta * dot;
            for (std::size_t j = start; j < Cols; ++j)
                vMatrix.at(i, j) -= scale * v.at(j, 0);
        }
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    OPTIMIZE_FOR_SPEED bool SingularValueDecomposition<T, Rows, Cols>::Decompose(
        const math::Matrix<T, Rows, Cols>& a)
    {
        math::Matrix<T, Rows, Cols> bidiag = a;

        math::Vector<T, Rows> lvec{};
        math::Vector<T, Cols> rvec{};

        std::array<math::Vector<T, Rows>, Cols> leftVecs{};
        std::array<math::Vector<T, Cols>, Cols> rightVecs{};
        std::array<T, Cols> leftBeta{};
        std::array<T, Cols> rightBeta{};

        for (std::size_t k = 0; k < Cols; ++k)
        {
            for (std::size_t i = 0; i < Rows; ++i)
                lvec.at(i, 0) = (i >= k) ? bidiag.at(i, k) : T{};

            T lbeta{};
            math::HouseholderVector(lvec, k, lvec, lbeta);
            leftBeta[k] = lbeta;
            leftVecs[k] = lvec;

            if (lbeta != T{})
                ApplyLeftReflector(bidiag, lvec, lbeta, k);

            if (k + 1 < Cols)
            {
                for (std::size_t j = 0; j < Cols; ++j)
                    rvec.at(j, 0) = (j >= k + 1) ? bidiag.at(k, j) : T{};

                T rbeta{};
                math::HouseholderVector(rvec, k + 1, rvec, rbeta);
                rightBeta[k] = rbeta;
                rightVecs[k] = rvec;

                if (rbeta != T{})
                    ApplyRightReflector(bidiag, rvec, rbeta, k + 1);
            }
            else
            {
                rightBeta[k] = T{};
            }
        }

        for (std::size_t i = 0; i < Cols; ++i)
            sigma.at(i, 0) = bidiag.at(i, i);

        for (std::size_t i = 0; i < Cols; ++i)
            superdiag.at(i, 0) = (i + 1 < Cols) ? bidiag.at(i, i + 1) : T{};

        for (std::size_t i = 0; i < Rows; ++i)
            for (std::size_t j = 0; j < Cols; ++j)
                uMat.at(i, j) = (i == j) ? T{ 1 } : T{};

        for (std::size_t k = Cols; k > 0; --k)
        {
            std::size_t col = k - 1;
            T lbeta = leftBeta[col];
            if (lbeta == T{})
                continue;

            AccumulateLeft(uMat, leftVecs[col], lbeta, col);
        }

        for (std::size_t i = 0; i < Cols; ++i)
            for (std::size_t j = 0; j < Cols; ++j)
                vMat.at(i, j) = (i == j) ? T{ 1 } : T{};

        for (std::size_t row = 0; row + 1 < Cols; ++row)
        {
            std::size_t k = row + 1;
            T rbeta = rightBeta[row];
            if (rbeta == T{})
                continue;

            AccumulateRight(vMat, rightVecs[row], rbeta, k);
        }

        constexpr std::size_t maxIter = 30 * Cols * Cols;

        for (std::size_t iter = 0; iter < maxIter; ++iter)
        {
            for (std::size_t i = 0; i < Cols - 1; ++i)
            {
                T thresh = T{ 1e-6f } * (std::abs(sigma.at(i, 0)) + std::abs(sigma.at(i + 1, 0)));
                if (std::abs(superdiag.at(i, 0)) <= thresh)
                    superdiag.at(i, 0) = T{};
            }

            std::size_t right = Cols - 1;
            while (right > 0)
            {
                T thresh = T{ 1e-6f } * (std::abs(sigma.at(right - 1, 0)) + std::abs(sigma.at(right, 0)));
                if (std::abs(superdiag.at(right - 1, 0)) > thresh)
                    break;
                superdiag.at(right - 1, 0) = T{};
                --right;
            }

            if (right == 0)
                break;

            std::size_t left = right - 1;
            while (left > 0)
            {
                T thresh = T{ 1e-6f } * (std::abs(sigma.at(left - 1, 0)) + std::abs(sigma.at(left, 0)));
                if (std::abs(superdiag.at(left - 1, 0)) <= thresh)
                    break;
                --left;
            }

            QrSweep(left, right - 1);
        }

        for (std::size_t i = 0; i < Cols; ++i)
        {
            if (sigma.at(i, 0) < T{})
            {
                sigma.at(i, 0) = -sigma.at(i, 0);
                for (std::size_t k = 0; k < Rows; ++k)
                    uMat.at(k, i) = -uMat.at(k, i);
            }
        }

        for (std::size_t i = 0; i < Cols - 1; ++i)
        {
            std::size_t maxIdx = i;
            for (std::size_t j = i + 1; j < Cols; ++j)
                if (sigma.at(j, 0) > sigma.at(maxIdx, 0))
                    maxIdx = j;

            if (maxIdx != i)
            {
                T tmp = sigma.at(i, 0);
                sigma.at(i, 0) = sigma.at(maxIdx, 0);
                sigma.at(maxIdx, 0) = tmp;

                for (std::size_t k = 0; k < Rows; ++k)
                {
                    T tu = uMat.at(k, i);
                    uMat.at(k, i) = uMat.at(k, maxIdx);
                    uMat.at(k, maxIdx) = tu;
                }
                for (std::size_t k = 0; k < Cols; ++k)
                {
                    T tv = vMat.at(k, i);
                    vMat.at(k, i) = vMat.at(k, maxIdx);
                    vMat.at(k, maxIdx) = tv;
                }
            }
        }

        return true;
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    OPTIMIZE_FOR_SPEED void SingularValueDecomposition<T, Rows, Cols>::QrSweep(
        std::size_t p, std::size_t q)
    {
        T dq = sigma.at(q, 0);
        T eq = superdiag.at(q, 0);
        T dqp1 = sigma.at(q + 1, 0);

        T t11 = dq * dq;
        T t12 = dq * eq;
        T t22 = eq * eq + dqp1 * dqp1;
        T half = (t11 - t22) / T{ 2 };
        T mu = t22 - t12 * t12 / (half + std::copysign(std::sqrt(half * half + t12 * t12), half));

        T f = sigma.at(p, 0) * sigma.at(p, 0) - mu;
        T g = sigma.at(p, 0) * superdiag.at(p, 0);

        for (std::size_t k = p; k <= q; ++k)
        {
            math::GivensRotation<T> gv = math::ComputeGivens(f, g);

            for (std::size_t i = 0; i < Cols; ++i)
                math::ApplyGivens(gv, vMat.at(i, k), vMat.at(i, k + 1));

            T dk = sigma.at(k, 0);
            T ek = superdiag.at(k, 0);
            T dkp1 = sigma.at(k + 1, 0);

            if (k > p)
                superdiag.at(k - 1, 0) = std::sqrt(f * f + g * g);

            f = gv.c * dk + gv.s * ek;
            ek = gv.c * ek - gv.s * dk;
            g = gv.s * dkp1;
            dkp1 = gv.c * dkp1;

            math::GivensRotation<T> gu = math::ComputeGivens(f, g);

            for (std::size_t i = 0; i < Rows; ++i)
                math::ApplyGivens(gu, uMat.at(i, k), uMat.at(i, k + 1));

            sigma.at(k, 0) = std::sqrt(f * f + g * g);

            f = gu.c * ek + gu.s * dkp1;
            dkp1 = gu.c * dkp1 - gu.s * ek;

            if (k + 1 < Cols - 1)
            {
                T ekp1 = superdiag.at(k + 1, 0);
                g = gu.s * ekp1;
                superdiag.at(k + 1, 0) = gu.c * ekp1;
            }

            sigma.at(k + 1, 0) = dkp1;
        }

        superdiag.at(q, 0) = f;
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    const math::Vector<T, Cols>& SingularValueDecomposition<T, Rows, Cols>::SingularValues() const
    {
        return sigma;
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    const math::Matrix<T, Rows, Cols>& SingularValueDecomposition<T, Rows, Cols>::U() const
    {
        return uMat;
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    const math::Matrix<T, Cols, Cols>& SingularValueDecomposition<T, Rows, Cols>::V() const
    {
        return vMat;
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    math::Matrix<T, Cols, Rows> SingularValueDecomposition<T, Rows, Cols>::PseudoInverse(T tol) const
    {
        math::Matrix<T, Cols, Cols> sigmaInv{};
        for (std::size_t i = 0; i < Cols; ++i)
            sigmaInv.at(i, i) = (sigma.at(i, 0) > tol) ? T{ 1 } / sigma.at(i, 0) : T{};

        return vMat * sigmaInv * uMat.Transpose();
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    std::size_t SingularValueDecomposition<T, Rows, Cols>::Rank(T tol) const
    {
        std::size_t r{ 0 };
        for (std::size_t i = 0; i < Cols; ++i)
            if (sigma.at(i, 0) > tol)
                ++r;
        return r;
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    T SingularValueDecomposition<T, Rows, Cols>::ConditionNumber() const
    {
        T sMin = sigma.at(Cols - 1, 0);
        if (sMin <= T{})
            return T{};
        return sigma.at(0, 0) / sMin;
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    math::Vector<T, Cols> SingularValueDecomposition<T, Rows, Cols>::SolveLeastSquares(
        const math::Vector<T, Rows>& b) const
    {
        math::Vector<T, Cols> utb{};
        for (std::size_t i = 0; i < Cols; ++i)
        {
            T dot{ T{} };
            for (std::size_t k = 0; k < Rows; ++k)
                dot += uMat.at(k, i) * b.at(k, 0);
            utb.at(i, 0) = (sigma.at(i, 0) > T{ 1e-10f }) ? dot / sigma.at(i, 0) : T{};
        }

        math::Vector<T, Cols> x{};
        for (std::size_t i = 0; i < Cols; ++i)
        {
            T dot{ T{} };
            for (std::size_t k = 0; k < Cols; ++k)
                dot += vMat.at(i, k) * utb.at(k, 0);
            x.at(i, 0) = dot;
        }

        return x;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class SingularValueDecomposition<float, 4, 3>;
    extern template class SingularValueDecomposition<float, 3, 3>;
#endif
}
