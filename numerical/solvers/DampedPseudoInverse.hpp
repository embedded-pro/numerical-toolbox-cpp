#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/solvers/SingularValueDecomposition.hpp"
#include <cstddef>
#include <type_traits>

namespace solvers
{
    template<typename T, std::size_t Rows, std::size_t Cols>
    class DampedPseudoInverse
    {
        static_assert(std::is_floating_point_v<T>, "DampedPseudoInverse supports floating-point types only");

        using Decomposition = SingularValueDecomposition<T, Rows, Cols>;
        static constexpr std::size_t singularValueCount{ Decomposition::SingularValueCount };

    public:
        explicit DampedPseudoInverse(T damping, T rankTolerance = T{ 1e-4f });

        bool Factor(const math::Matrix<T, Rows, Cols>& a);
        OPTIMIZE_FOR_SPEED math::Vector<T, Cols> Solve(const math::Vector<T, Rows>& b) const;
        OPTIMIZE_FOR_SPEED math::Vector<T, Cols> Project(const math::Vector<T, Cols>& x) const;
        math::Matrix<T, Cols, Rows> Inverse() const;
        math::SquareMatrix<T, Cols> NullSpaceProjector() const;
        std::size_t Rank() const;

    private:
        T Gain(T sigma) const;

        Decomposition svd;
        T damping;
        T rankTolerance;
        T threshold{};
        std::size_t rank{ 0 };
    };

    template<typename T, std::size_t Rows, std::size_t Cols>
    DampedPseudoInverse<T, Rows, Cols>::DampedPseudoInverse(T damping, T rankTolerance)
        : damping{ damping }
        , rankTolerance{ rankTolerance }
    {
        really_assert(damping >= T{});
        really_assert(rankTolerance >= T{});
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    bool DampedPseudoInverse<T, Rows, Cols>::Factor(const math::Matrix<T, Rows, Cols>& a)
    {
        rank = 0;
        if (!svd.Decompose(a))
            return false;

        const auto& sigma{ svd.SingularValues() };
        threshold = rankTolerance * sigma[0];
        while (rank < singularValueCount && sigma[rank] > threshold)
            ++rank;

        return true;
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    OPTIMIZE_FOR_SPEED math::Vector<T, Cols> DampedPseudoInverse<T, Rows, Cols>::Solve(const math::Vector<T, Rows>& b) const
    {
        const auto& sigma{ svd.SingularValues() };
        const auto& u{ svd.U() };
        const auto& v{ svd.V() };

        math::Vector<T, Cols> x{};
        for (std::size_t i = 0; i < singularValueCount; ++i)
        {
            T projection{};
            for (std::size_t r = 0; r < Rows; ++r)
                projection += u.at(r, i) * b[r];

            const T weight{ Gain(sigma[i]) * projection };
            for (std::size_t c = 0; c < Cols; ++c)
                x[c] += weight * v.at(c, i);
        }

        return x;
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    OPTIMIZE_FOR_SPEED math::Vector<T, Cols> DampedPseudoInverse<T, Rows, Cols>::Project(const math::Vector<T, Cols>& x) const
    {
        const auto& v{ svd.V() };

        math::Vector<T, Cols> projected{ x };
        for (std::size_t i = 0; i < rank; ++i)
        {
            T component{};
            for (std::size_t c = 0; c < Cols; ++c)
                component += v.at(c, i) * x[c];

            for (std::size_t c = 0; c < Cols; ++c)
                projected[c] -= component * v.at(c, i);
        }

        return projected;
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    math::Matrix<T, Cols, Rows> DampedPseudoInverse<T, Rows, Cols>::Inverse() const
    {
        const auto& sigma{ svd.SingularValues() };
        const auto& u{ svd.U() };
        const auto& v{ svd.V() };

        math::Matrix<T, Cols, Rows> inverse{};
        for (std::size_t i = 0; i < singularValueCount; ++i)
        {
            const T gain{ Gain(sigma[i]) };
            for (std::size_t c = 0; c < Cols; ++c)
                for (std::size_t r = 0; r < Rows; ++r)
                    inverse.at(c, r) += gain * v.at(c, i) * u.at(r, i);
        }

        return inverse;
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    math::SquareMatrix<T, Cols> DampedPseudoInverse<T, Rows, Cols>::NullSpaceProjector() const
    {
        const auto& v{ svd.V() };

        auto projector{ math::SquareMatrix<T, Cols>::Identity() };
        for (std::size_t i = 0; i < rank; ++i)
            for (std::size_t r = 0; r < Cols; ++r)
                for (std::size_t c = 0; c < Cols; ++c)
                    projector.at(r, c) -= v.at(r, i) * v.at(c, i);

        return projector;
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    std::size_t DampedPseudoInverse<T, Rows, Cols>::Rank() const
    {
        return rank;
    }

    template<typename T, std::size_t Rows, std::size_t Cols>
    T DampedPseudoInverse<T, Rows, Cols>::Gain(T sigma) const
    {
        if (damping > T{})
            return sigma / (sigma * sigma + damping * damping);

        return sigma > threshold ? T{ 1 } / sigma : T{};
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class DampedPseudoInverse<float, 2, 3>;
    extern template class DampedPseudoInverse<float, 6, 7>;
#endif
}
