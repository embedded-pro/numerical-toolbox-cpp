#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/GivensRotation.hpp"
#include "numerical/math/HouseholderTransform.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include <array>
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
        using LeftVectors = std::array<math::Vector<T, Rows>, Cols>;
        using RightVectors = std::array<math::Vector<T, Cols>, Cols>;
        using Betas = std::array<T, Cols>;

        void Bidiagonalize(math::Matrix<T, Rows, Cols>& bidiag,
            LeftVectors& leftVecs, Betas& leftBeta, RightVectors& rightVecs, Betas& rightBeta);
        void ExtractBidiagonal(const math::Matrix<T, Rows, Cols>& bidiag);
        void AccumulateU(const LeftVectors& leftVecs, const Betas& leftBeta);
        void AccumulateV(const RightVectors& rightVecs, const Betas& rightBeta);
        void DiagonalizeGolubKahan();
        bool NextBlock(std::size_t& p, std::size_t& q);
        T WilkinsonShift(std::size_t q) const;
        void QrSweep(std::size_t p, std::size_t q);
        void MakeSingularValuesPositive();
        void SortDescending();
        void SwapColumns(std::size_t i, std::size_t j);

        math::Matrix<T, Rows, Cols> uMat{};
        math::Vector<T, Cols> sigma{};
        math::Vector<T, Cols> superdiag{};
        math::Matrix<T, Cols, Cols> vMat{};
    };

    template<typename T, std::size_t Rows, std::size_t Cols>
    void SingularValueDecomposition<T, Rows, Cols>::Bidiagonalize(
        math::Matrix<T, Rows, Cols>& bidiag,
        LeftVectors& leftVecs, Betas& leftBeta, RightVectors& rightVecs, Betas& rightBeta)
    {
        math::Vector<T, Rows> lvec{};
        math::Vector<T, Cols> rvec{};

        for (std::size_t k = 0; k < Cols; ++k)
        {
            for (std::size_t i = 0; i < Rows; ++i)
                lvec.at(i, 0) = (i >= k) ? bidiag.at(i, k) : T{};

            T lbeta{};
            math::HouseholderVector(lvec, k, lvec, lbeta);
            leftBeta[k] = lbeta;
            leftVecs[k] = lvec;

            if (lbeta != T{})
                math::ApplyReflectorLeft(bidiag, lvec, lbeta, k, k);

            if (k + 1 < Cols)
            {
                for (std::size_t j = 0; j < Cols; ++j)
                    rvec.at(j, 0) = (j >= k + 1) ? bidiag.at(k, j) : T{};

                T rbeta{};
                math::HouseholderVector(rvec, k + 1, rvec, rbeta);
                rightBeta[k] = rbeta;
                rightVecs[k] = rvec;

                if (rbeta != T{})
                    math::ApplyReflectorRight(bidiag, rvec, rbeta, 0, k + 1);
            }
            else
            {
                rightBeta[k] = T{};
            }
        }
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    void SingularValueDecomposition<T, Rows, Cols>::ExtractBidiagonal(
        const math::Matrix<T, Rows, Cols>& bidiag)
    {
        for (std::size_t i = 0; i < Cols; ++i)
        {
            sigma.at(i, 0) = bidiag.at(i, i);
            superdiag.at(i, 0) = (i + 1 < Cols) ? bidiag.at(i, i + 1) : T{};
        }
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    void SingularValueDecomposition<T, Rows, Cols>::AccumulateU(
        const LeftVectors& leftVecs, const Betas& leftBeta)
    {
        for (std::size_t i = 0; i < Rows; ++i)
            for (std::size_t j = 0; j < Cols; ++j)
                uMat.at(i, j) = (i == j) ? T{ 1 } : T{};

        for (std::size_t k = Cols; k > 0; --k)
        {
            std::size_t col = k - 1;
            T lbeta = leftBeta[col];
            if (lbeta != T{})
                math::ApplyReflectorLeft(uMat, leftVecs[col], lbeta, col, 0);
        }
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    void SingularValueDecomposition<T, Rows, Cols>::AccumulateV(
        const RightVectors& rightVecs, const Betas& rightBeta)
    {
        for (std::size_t i = 0; i < Cols; ++i)
            for (std::size_t j = 0; j < Cols; ++j)
                vMat.at(i, j) = (i == j) ? T{ 1 } : T{};

        for (std::size_t row = 0; row + 1 < Cols; ++row)
        {
            T rbeta = rightBeta[row];
            if (rbeta != T{})
                math::ApplyReflectorRight(vMat, rightVecs[row], rbeta, 0, row + 1);
        }
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    OPTIMIZE_FOR_SPEED bool SingularValueDecomposition<T, Rows, Cols>::Decompose(
        const math::Matrix<T, Rows, Cols>& a)
    {
        math::Matrix<T, Rows, Cols> bidiag = a;

        LeftVectors leftVecs{};
        RightVectors rightVecs{};
        Betas leftBeta{};
        Betas rightBeta{};

        Bidiagonalize(bidiag, leftVecs, leftBeta, rightVecs, rightBeta);
        ExtractBidiagonal(bidiag);
        AccumulateU(leftVecs, leftBeta);
        AccumulateV(rightVecs, rightBeta);
        DiagonalizeGolubKahan();
        MakeSingularValuesPositive();
        SortDescending();

        return true;
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    bool SingularValueDecomposition<T, Rows, Cols>::NextBlock(std::size_t& p, std::size_t& q)
    {
        for (std::size_t i = 0; i < Cols - 1; ++i)
        {
            T thresh = T{ 1e-6f } * (math::Abs(sigma.at(i, 0)) + math::Abs(sigma.at(i + 1, 0)));
            if (math::Abs(superdiag.at(i, 0)) <= thresh)
                superdiag.at(i, 0) = T{};
        }

        std::size_t right = Cols - 1;
        while (right > 0)
        {
            T thresh = T{ 1e-6f } * (math::Abs(sigma.at(right - 1, 0)) + math::Abs(sigma.at(right, 0)));
            if (math::Abs(superdiag.at(right - 1, 0)) > thresh)
                break;
            superdiag.at(right - 1, 0) = T{};
            --right;
        }

        if (right == 0)
            return false;

        std::size_t left = right - 1;
        while (left > 0)
        {
            T thresh = T{ 1e-6f } * (math::Abs(sigma.at(left - 1, 0)) + math::Abs(sigma.at(left, 0)));
            if (math::Abs(superdiag.at(left - 1, 0)) <= thresh)
                break;
            --left;
        }

        p = left;
        q = right - 1;
        return true;
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    OPTIMIZE_FOR_SPEED void SingularValueDecomposition<T, Rows, Cols>::DiagonalizeGolubKahan()
    {
        constexpr std::size_t maxIter = 30 * Cols * Cols;

        std::size_t p{};
        std::size_t q{};
        for (std::size_t iter = 0; iter < maxIter; ++iter)
        {
            if (!NextBlock(p, q))
                return;
            QrSweep(p, q);
        }
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    T SingularValueDecomposition<T, Rows, Cols>::WilkinsonShift(std::size_t q) const
    {
        T dq = sigma.at(q, 0);
        T eq = superdiag.at(q, 0);
        T dqp1 = sigma.at(q + 1, 0);

        T t11 = dq * dq;
        T t12 = dq * eq;
        T t22 = eq * eq + dqp1 * dqp1;
        T half = (t11 - t22) / T{ 2 };
        return t22 - t12 * t12 / (half + math::Copysign(math::Sqrt(half * half + t12 * t12), half));
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    OPTIMIZE_FOR_SPEED void SingularValueDecomposition<T, Rows, Cols>::QrSweep(
        std::size_t p, std::size_t q)
    {
        T mu = (math::Abs(sigma.at(q + 1, 0)) <= T{ 1e-6f } * math::Abs(sigma.at(q, 0)))
                   ? T{}
                   : WilkinsonShift(q);

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
                superdiag.at(k - 1, 0) = math::Sqrt(f * f + g * g);

            f = gv.c * dk + gv.s * ek;
            ek = gv.c * ek - gv.s * dk;
            g = gv.s * dkp1;
            dkp1 = gv.c * dkp1;

            math::GivensRotation<T> gu = math::ComputeGivens(f, g);

            for (std::size_t i = 0; i < Rows; ++i)
                math::ApplyGivens(gu, uMat.at(i, k), uMat.at(i, k + 1));

            sigma.at(k, 0) = math::Sqrt(f * f + g * g);

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
    void SingularValueDecomposition<T, Rows, Cols>::MakeSingularValuesPositive()
    {
        for (std::size_t i = 0; i < Cols; ++i)
        {
            if (sigma.at(i, 0) < T{})
            {
                sigma.at(i, 0) = -sigma.at(i, 0);
                for (std::size_t k = 0; k < Rows; ++k)
                    uMat.at(k, i) = -uMat.at(k, i);
            }
        }
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    void SingularValueDecomposition<T, Rows, Cols>::SwapColumns(std::size_t i, std::size_t j)
    {
        T tmp = sigma.at(i, 0);
        sigma.at(i, 0) = sigma.at(j, 0);
        sigma.at(j, 0) = tmp;

        for (std::size_t k = 0; k < Rows; ++k)
        {
            T tu = uMat.at(k, i);
            uMat.at(k, i) = uMat.at(k, j);
            uMat.at(k, j) = tu;
        }
        for (std::size_t k = 0; k < Cols; ++k)
        {
            T tv = vMat.at(k, i);
            vMat.at(k, i) = vMat.at(k, j);
            vMat.at(k, j) = tv;
        }
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    void SingularValueDecomposition<T, Rows, Cols>::SortDescending()
    {
        for (std::size_t i = 0; i + 1 < Cols; ++i)
        {
            std::size_t maxIdx = i;
            for (std::size_t j = i + 1; j < Cols; ++j)
                if (sigma.at(j, 0) > sigma.at(maxIdx, 0))
                    maxIdx = j;

            if (maxIdx != i)
                SwapColumns(i, maxIdx);
        }
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
