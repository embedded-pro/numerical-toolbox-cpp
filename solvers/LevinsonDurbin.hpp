#ifndef SOLVERS_LEVINSON_DURBIN_HPP
#define SOLVERS_LEVINSON_DURBIN_HPP

#include "math/Toeplitz.hpp"
#include "solvers/Solver.hpp"

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
    };

    // Implementation //

    template<typename T, std::size_t N>
    typename LevinsonDurbin<T, N>::SolutionVector
    LevinsonDurbin<T, N>::Solve(const InputMatrix& A, const InputVector& b)
    {
        really_assert(IsToeplitz(A));

        auto [first_row, first_col] = math::ToeplitzMatrix<T, N>::ExtractToeplitzVectors(A);
        auto toeplitz = math::ToeplitzMatrix<T, N>(first_row, first_col);

        SolutionVector phi;
        SolutionVector prev_phi;
        SolutionVector k;

        T prev_error = first_row.at(0, 0);
        T curr_error;

        k.at(0, 0) = b.at(0, 0) / prev_error;
        phi.at(0, 0) = k.at(0, 0);
        prev_error *= (T(1.0f) - k.at(0, 0) * k.at(0, 0));

        for (size_t n = 1; n < N; ++n)
        {
            T alpha = b.at(n, 0);

            for (size_t j = 0; j < n; ++j)
                alpha -= phi.at(j, 0) * first_row.at(n - j, 0);

            k.at(n, 0) = alpha / prev_error;

            for (size_t j = 0; j < n; ++j)
                prev_phi.at(j, 0) = phi.at(j, 0);

            for (size_t j = 0; j <= n; ++j)
                if (j < n)
                    phi.at(j, 0) = prev_phi.at(j, 0) + k.at(n, 0) * prev_phi.at(n - 1 - j, 0);
                else
                    phi.at(n, 0) = k.at(n, 0);

            prev_error *= (T(1.0f) - k.at(n, 0) * k.at(n, 0));
        }

        return phi;
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
}

#endif
