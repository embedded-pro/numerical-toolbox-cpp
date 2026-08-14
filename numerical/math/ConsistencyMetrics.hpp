#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CholeskyDecomposition.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/MatrixNorms.hpp"
#include <array>
#include <cstddef>
#include <optional>

namespace math
{
    namespace detail
    {
        static constexpr std::size_t kMaxChiSquareDim = 10;

        static constexpr std::array<float, kMaxChiSquareDim> kChi2Lo95 = {
            0.000982f, 0.050636f, 0.215795f, 0.484419f, 0.831212f,
            1.237344f, 1.689869f, 2.179731f, 2.700389f, 3.246973f
        };

        static constexpr std::array<float, kMaxChiSquareDim> kChi2Hi95 = {
            5.023886f, 7.377759f, 9.348404f, 11.143480f, 12.832502f,
            14.449376f, 16.012764f, 17.534546f, 19.022768f, 20.483177f
        };

        static constexpr float kZ975 = 1.95996f;

        [[nodiscard]] OPTIMIZE_FOR_SPEED constexpr float Chi2QuantileApprox(float z, float dof)
        {
            const float mu = 1.0f - 2.0f / (9.0f * dof);
            const float sigma = math::Sqrt(2.0f / (9.0f * dof));
            const float x = mu + z * sigma;
            return dof * x * x * x;
        }
    }

    template<typename T, std::size_t Dim>
    class ConsistencyMetrics
    {
        static_assert(std::is_floating_point_v<T>, "ConsistencyMetrics supports floating-point types");
        static_assert(Dim >= 1 && Dim <= detail::kMaxChiSquareDim,
            "ConsistencyMetrics Dim must be in [1, kMaxChiSquareDim]");

    public:
        using StateVector = math::Vector<T, Dim>;
        using CovarianceMatrix = math::Matrix<T, Dim, Dim>;

        [[nodiscard]] static OPTIMIZE_FOR_SPEED std::optional<T> Nees(const StateVector& error, const CovarianceMatrix& covariance);
        [[nodiscard]] static OPTIMIZE_FOR_SPEED std::optional<T> Nis(const StateVector& innovation, const CovarianceMatrix& innovationCovariance);
        [[nodiscard]] static bool IsConsistent(T value);
        [[nodiscard]] static bool IsTimeAveragedConsistent(T averagedValue, std::size_t numSamples);
    };

    template<typename T, std::size_t Dim>
    OPTIMIZE_FOR_SPEED std::optional<T> ConsistencyMetrics<T, Dim>::Nees(const StateVector& error, const CovarianceMatrix& covariance)
    {
        auto z = CholeskyDecomposition<T, Dim>::Solve(covariance, error);
        if (!z.has_value())
            return std::nullopt;
        return math::DotProduct(error, z.value());
    }

    template<typename T, std::size_t Dim>
    OPTIMIZE_FOR_SPEED std::optional<T> ConsistencyMetrics<T, Dim>::Nis(const StateVector& innovation, const CovarianceMatrix& innovationCovariance)
    {
        auto z = CholeskyDecomposition<T, Dim>::Solve(innovationCovariance, innovation);
        if (!z.has_value())
            return std::nullopt;
        return math::DotProduct(innovation, z.value());
    }

    template<typename T, std::size_t Dim>
    bool ConsistencyMetrics<T, Dim>::IsConsistent(T value)
    {
        return value >= static_cast<T>(detail::kChi2Lo95[Dim - 1]) &&
               value <= static_cast<T>(detail::kChi2Hi95[Dim - 1]);
    }

    template<typename T, std::size_t Dim>
    bool ConsistencyMetrics<T, Dim>::IsTimeAveragedConsistent(T averagedValue, std::size_t numSamples)
    {
        if (numSamples == 0)
            return false;
        const std::size_t dof = numSamples * Dim;
        const auto fdof = static_cast<float>(dof);
        const auto fsamples = static_cast<float>(numSamples);
        const float lo_raw = (dof <= detail::kMaxChiSquareDim)
                                 ? detail::kChi2Lo95[dof - 1]
                                 : detail::Chi2QuantileApprox(-detail::kZ975, fdof);
        const float hi_raw = (dof <= detail::kMaxChiSquareDim)
                                 ? detail::kChi2Hi95[dof - 1]
                                 : detail::Chi2QuantileApprox(detail::kZ975, fdof);
        const float lo = lo_raw / fsamples;
        const float hi = hi_raw / fsamples;
        return static_cast<float>(averagedValue) >= lo &&
               static_cast<float>(averagedValue) <= hi;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class ConsistencyMetrics<float, 1>;
    extern template class ConsistencyMetrics<float, 2>;
    extern template class ConsistencyMetrics<float, 3>;
#endif
}
