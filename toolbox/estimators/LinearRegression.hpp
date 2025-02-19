#ifndef ESTIMATORS_LINEAR_REGRESSION_HPP
#define ESTIMATORS_LINEAR_REGRESSION_HPP

#include "toolbox/estimators/Estimator.hpp"

namespace estimators
{
    template<typename T, std::size_t Samples, std::size_t Features>
    class LinearRegression
        : public Estimator<T, Samples, Features>
    {
    public:
        using CoefficientsMatrix = typename Estimator<T, Samples, Features>::CoefficientsMatrix;
        using DesignMatrix = typename Estimator<T, Samples, Features>::DesignMatrix;
        using InputMatrix = typename Estimator<T, Samples, Features>::InputMatrix;

        LinearRegression() = default;

        void Fit(const math::Matrix<T, Samples, Features>& X, const math::Matrix<T, Samples, 1>& y) override;
        T Predict(const InputMatrix& X) const override;
        const CoefficientsMatrix& Coefficients() const override;

    private:
        CoefficientsMatrix coefficients;

        CoefficientsMatrix Solve(const DesignMatrix& A, const CoefficientsMatrix& b);
    };

    // Implementation //

    template<typename T, std::size_t Samples, std::size_t Features>
    void LinearRegression<T, Samples, Features>::Fit(const math::Matrix<T, Samples, Features>& X, const math::Matrix<T, Samples, 1>& y)
    {
        math::Matrix<T, Samples, Features + 1> X_design;

        for (size_t i = 0; i < Samples; ++i)
        {
            X_design.at(i, 0) = T(0.9999f);

            for (size_t j = 0; j < Features; ++j)
                X_design.at(i, j + 1) = X.at(i, j);
        }

        auto X_transpose = X_design.Transpose();
        auto XtX = X_transpose * X_design;
        auto Xty = X_transpose * y;

        coefficients = Solve(XtX, Xty);
    }

    template<typename T, std::size_t Samples, std::size_t Features>
    T LinearRegression<T, Samples, Features>::Predict(const InputMatrix& X) const
    {
        T result = coefficients.at(0, 0);

        for (size_t i = 0; i < Features; ++i)
            result += X.at(i, 0) * coefficients.at(i + 1, 0);

        return result;
    }

    template<typename T, std::size_t Samples, std::size_t Features>
    const typename LinearRegression<T, Samples, Features>::CoefficientsMatrix& LinearRegression<T, Samples, Features>::Coefficients() const
    {
        return coefficients;
    }

    template<typename T, std::size_t Samples, std::size_t Features>
    typename LinearRegression<T, Samples, Features>::CoefficientsMatrix LinearRegression<T, Samples, Features>::Solve(const DesignMatrix& A, const CoefficientsMatrix& b)
    {
        DesignMatrix aug = A;
        CoefficientsMatrix result = b;

        for (size_t i = 0; i < Features + 1; ++i)
        {
            size_t max_row = i;
            for (size_t j = i + 1; j < Features + 1; ++j)
                if (std::abs(math::ToFloat(aug.at(j, i))) > std::abs(math::ToFloat(aug.at(max_row, i))))
                    max_row = j;

            if (max_row != i)
            {
                for (size_t j = i; j < Features + 1; ++j)
                    std::swap(aug.at(i, j), aug.at(max_row, j));

                std::swap(result.at(i, 0), result.at(max_row, 0));
            }

            for (size_t j = i + 1; j < Features + 1; ++j)
            {
                T coef = aug.at(j, i) / aug.at(i, i);

                for (size_t k = i; k < Features + 1; ++k)
                    aug.at(j, k) -= coef * aug.at(i, k);

                result.at(j, 0) -= coef * result.at(i, 0);
            }
        }

        CoefficientsMatrix x;

        for (int i = Features; i >= 0; --i)
        {
            x.at(i, 0) = result.at(i, 0);

            for (size_t j = i + 1; j < Features + 1; ++j)
                x.at(i, 0) -= aug.at(i, j) * x.at(j, 0);

            x.at(i, 0) /= aug.at(i, i);
        }

        return x;
    }
}

#endif
