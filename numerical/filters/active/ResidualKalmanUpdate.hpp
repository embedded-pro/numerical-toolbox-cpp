#pragma once

#include "numerical/math/CholeskyDecomposition.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/ConsistencyMetrics.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/MatrixOperations.hpp"
#include "numerical/math/TriangularSolve.hpp"
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <type_traits>

namespace filters
{
    enum class KalmanUpdateStatus : std::uint8_t
    {
        Applied,
        Gated,
        Singular
    };

    template<typename T>
    struct KalmanUpdateResult
    {
        static_assert(std::is_floating_point_v<T>, "KalmanUpdateResult supports floating-point types");

        KalmanUpdateStatus status{ KalmanUpdateStatus::Singular };
        T nis{};
    };

    template<typename T, std::size_t StateSize, std::size_t MeasurementSize>
    class ResidualKalmanUpdate
    {
        static_assert(std::is_floating_point_v<T>, "ResidualKalmanUpdate supports floating-point types");
        static_assert(StateSize > 0 && MeasurementSize > 0, "ResidualKalmanUpdate needs a state and a measurement");

    public:
        using StateVector = math::Vector<T, StateSize>;
        using Covariance = math::SquareMatrix<T, StateSize>;
        using Residual = math::Vector<T, MeasurementSize>;
        using Jacobian = math::Matrix<T, MeasurementSize, StateSize>;
        using MeasurementNoise = math::SquareMatrix<T, MeasurementSize>;

        static std::optional<T> Nis(const Residual& residual, const Jacobian& jacobian, const Covariance& covariance, const MeasurementNoise& noise);
        OPTIMIZE_FOR_SPEED static KalmanUpdateResult<T> Apply(StateVector& state, Covariance& covariance, const Residual& residual, const Jacobian& jacobian,
            const MeasurementNoise& noise, T gate = std::numeric_limits<T>::infinity());

    private:
        using Factor = math::SquareMatrix<T, MeasurementSize>;
        using Gain = math::Matrix<T, StateSize, MeasurementSize>;

        static Residual Solve(const Factor& lower, const Factor& upper, const Residual& rhs);
    };

    template<typename T, std::size_t StateSize, std::size_t MeasurementSize>
    std::optional<T> ResidualKalmanUpdate<T, StateSize, MeasurementSize>::Nis(const Residual& residual, const Jacobian& jacobian, const Covariance& covariance, const MeasurementNoise& noise)
    {
        return math::ConsistencyMetrics<T, MeasurementSize>::Nis(residual, math::CongruenceTransform(jacobian, covariance) + noise);
    }

    template<typename T, std::size_t StateSize, std::size_t MeasurementSize>
    OPTIMIZE_FOR_SPEED KalmanUpdateResult<T> ResidualKalmanUpdate<T, StateSize, MeasurementSize>::Apply(StateVector& state, Covariance& covariance, const Residual& residual,
        const Jacobian& jacobian, const MeasurementNoise& noise, T gate)
    {
        const auto innovationCovariance{ math::CongruenceTransform(jacobian, covariance) + noise };
        const auto lower{ math::CholeskyDecomposition<T, MeasurementSize>::TryFactor(innovationCovariance) };
        if (!lower)
            return KalmanUpdateResult<T>{ KalmanUpdateStatus::Singular, T{} };

        const Factor upper{ lower->Transpose() };
        const T nis{ math::DotProduct(residual, Solve(*lower, upper, residual)) };
        if (nis > gate)
            return KalmanUpdateResult<T>{ KalmanUpdateStatus::Gated, nis };

        const auto jacobianCovariance{ jacobian * covariance };
        Gain gain{};
        for (std::size_t row = 0; row < StateSize; ++row)
            gain.SetBlock(Solve(*lower, upper, jacobianCovariance.template GetBlock<MeasurementSize, 1>(0, row)).Transpose(), row, 0);

        state = state + gain * residual;
        const auto correction{ Covariance::Identity() - gain * jacobian };
        covariance = math::Symmetrize(math::CongruenceTransform(correction, covariance) + math::CongruenceTransform(gain, noise));
        return KalmanUpdateResult<T>{ KalmanUpdateStatus::Applied, nis };
    }

    template<typename T, std::size_t StateSize, std::size_t MeasurementSize>
    auto ResidualKalmanUpdate<T, StateSize, MeasurementSize>::Solve(const Factor& lower, const Factor& upper, const Residual& rhs) -> Residual
    {
        return math::SolveUpperTriangular(upper, math::SolveLowerTriangular(lower, rhs));
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class ResidualKalmanUpdate<float, 1, 1>;
    extern template class ResidualKalmanUpdate<float, 2, 1>;
    extern template class ResidualKalmanUpdate<float, 2, 2>;
    extern template class ResidualKalmanUpdate<float, 3, 2>;
    extern template class ResidualKalmanUpdate<float, 4, 1>;
#endif
}
