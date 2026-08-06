#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Toeplitz.hpp"
#include "numerical/solvers/Solver.hpp"

namespace solvers
{
    template<typename T, std::size_t N>
    class LevinsonDurbin
        : public Solver<T, N>
    {
    public:
        using typename Solver<T, N>::SolutionVector;
        using typename Solver<T, N>::InputMatrix;
        using typename Solver<T, N>::InputVector;

        LevinsonDurbin() = default;
        SolutionVector Solve(const InputMatrix& A, const InputVector& b) override;

    private:
        bool IsToeplitz(const InputMatrix& A) const;
        static T WeightedTail(const InputVector& r, const SolutionVector& v, size_t n);
    };

    // Implementation //

    template<typename T, std::size_t N>
    typename LevinsonDurbin<T, N>::SolutionVector
        OPTIMIZE_FOR_SPEED
        LevinsonDurbin<T, N>::Solve(const InputMatrix& A, const InputVector& b)
    {
        really_assert(IsToeplitz(A));

        auto first_row = math::ToeplitzMatrix<T, N>::ExtractToeplitzVectors(A).first;

        SolutionVector x;
        SolutionVector y;
        SolutionVector prev_y;

        const T r0 = first_row.at(0, 0);
        T error = r0;
        T reflection = T(0.0f);

        x.at(0, 0) = b.at(0, 0) / r0;

        if constexpr (N > 1)
        {
            y.at(0, 0) = -first_row.at(1, 0) / r0;
            reflection = y.at(0, 0);
        }

        for (size_t n = 1; n < N; ++n)
        {
            error *= (T(1.0f) - reflection * reflection);

            const T mu = (b.at(n, 0) - WeightedTail(first_row, x, n)) / error;

            for (size_t j = 0; j < n; ++j)
                x.at(j, 0) += mu * y.at(n - 1 - j, 0);
            x.at(n, 0) = mu;

            if (n + 1 < N)
            {
                reflection = -(first_row.at(n + 1, 0) + WeightedTail(first_row, y, n)) / error;

                for (size_t j = 0; j < n; ++j)
                    prev_y.at(j, 0) = y.at(j, 0);
                for (size_t j = 0; j < n; ++j)
                    y.at(j, 0) = prev_y.at(j, 0) + reflection * prev_y.at(n - 1 - j, 0);
                y.at(n, 0) = reflection;
            }
        }

        return x;
    }

    template<typename T, std::size_t N>
    T LevinsonDurbin<T, N>::WeightedTail(const InputVector& r, const SolutionVector& v, size_t n)
    {
        T acc = T(0.0f);
        for (size_t i = 1; i <= n; ++i)
            acc += r.at(i, 0) * v.at(n - i, 0);
        return acc;
    }

    template<typename T, std::size_t N>
    bool LevinsonDurbin<T, N>::IsToeplitz(const InputMatrix& A) const
    {
        return math::ToeplitzMatrix<T, N>::IsToeplitzMatrix(A);
    }

    template<typename T, std::size_t N>
    [[nodiscard]] constexpr LevinsonDurbin<T, N> MakeLevinsonDurbin()
    {
        return LevinsonDurbin<T, N>();
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class LevinsonDurbin<float, 2>;
    extern template class LevinsonDurbin<float, 3>;
#endif
}
