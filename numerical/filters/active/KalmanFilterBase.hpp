#pragma once

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/solvers/GaussianElimination.hpp"

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

namespace filters
{
    namespace detail
    {
        template<typename QNumberType>
        inline constexpr bool is_kalman_supported_v =
            std::disjunction_v<
                std::is_same<QNumberType, float>,
                std::is_same<QNumberType, math::Q15>,
                std::is_same<QNumberType, math::Q31>>;
    }

    namespace detail
    {
        template<typename T, std::size_t Rows, std::size_t Cols>
        struct ControlType
        {
            using Matrix = math::Matrix<T, Rows, Cols>;
            using Vector = math::Vector<T, Cols>;
        };

        template<typename T, std::size_t Rows>
        struct ControlType<T, Rows, 0>
        {
            struct Matrix
            {};

            struct Vector
            {};
        };
    }

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize = 0>
    class KalmanFilterBase
    {
        static_assert(detail::is_kalman_supported_v<QNumberType>,
            "KalmanFilter only supports float, Q15, or Q31 types");
        static_assert(math::detail::is_valid_dimensions_v<StateSize, StateSize>,
            "State dimensions must be positive");
        static_assert(math::detail::is_valid_dimensions_v<MeasurementSize, MeasurementSize>,
            "Measurement dimensions must be positive");

    public:
        using StateMatrix = math::SquareMatrix<QNumberType, StateSize>;
        using StateVector = math::Vector<QNumberType, StateSize>;
        using MeasurementMatrix = math::Matrix<QNumberType, MeasurementSize, StateSize>;
        using MeasurementVector = math::Vector<QNumberType, MeasurementSize>;
        using MeasurementCovariance = math::SquareMatrix<QNumberType, MeasurementSize>;
        using KalmanGain = math::Matrix<QNumberType, StateSize, MeasurementSize>;
        using ControlMatrix = typename detail::ControlType<QNumberType, StateSize, ControlSize>::Matrix;
        using ControlVector = typename detail::ControlType<QNumberType, StateSize, ControlSize>::Vector;

        void SetProcessNoise(const StateMatrix& Q);
        void SetMeasurementNoise(const MeasurementCovariance& R);

        [[nodiscard]] const StateVector& GetState() const;
        [[nodiscard]] const StateMatrix& GetCovariance() const;

    protected:
        KalmanFilterBase(const StateVector& initialState, const StateMatrix& initialCovariance);

        KalmanGain ComputeKalmanGain(const MeasurementMatrix& H) const;
        void UpdateState(const MeasurementVector& innovation, const KalmanGain& K, const MeasurementMatrix& H);

        StateVector& state()
        {
            return state_;
        }

        const StateVector& state() const
        {
            return state_;
        }

        StateMatrix& covariance()
        {
            return covariance_;
        }

        const StateMatrix& covariance() const
        {
            return covariance_;
        }

        const StateMatrix& processNoise() const
        {
            return processNoise_;
        }

        const MeasurementCovariance& measurementNoise() const
        {
            return measurementNoise_;
        }

    private:
        StateVector state_;
        StateMatrix covariance_;
        StateMatrix processNoise_ = StateMatrix::Identity();
        MeasurementCovariance measurementNoise_ = MeasurementCovariance::Identity();
    };

    // Implementation //

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    KalmanFilterBase<QNumberType, StateSize, MeasurementSize, ControlSize>::KalmanFilterBase(
        const StateVector& initialState, const StateMatrix& initialCovariance)
        : state_(initialState)
        , covariance_(initialCovariance)
    {}

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    void KalmanFilterBase<QNumberType, StateSize, MeasurementSize, ControlSize>::SetProcessNoise(const StateMatrix& Q)
    {
        processNoise_ = Q;
    }

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    void KalmanFilterBase<QNumberType, StateSize, MeasurementSize, ControlSize>::SetMeasurementNoise(const MeasurementCovariance& R)
    {
        measurementNoise_ = R;
    }

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    OPTIMIZE_FOR_SPEED
        typename KalmanFilterBase<QNumberType, StateSize, MeasurementSize, ControlSize>::KalmanGain
        KalmanFilterBase<QNumberType, StateSize, MeasurementSize, ControlSize>::ComputeKalmanGain(const MeasurementMatrix& H) const
    {
        auto S = H * covariance_ * H.Transpose() + measurementNoise_;

        // Solve S Kᵀ = H P instead of explicit K = P Hᵀ S⁻¹ (S is symmetric)
        auto KTranspose = solvers::SolveSystem<QNumberType, MeasurementSize, StateSize>(S, H * covariance_);

        return KTranspose.Transpose();
    }

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    OPTIMIZE_FOR_SPEED void KalmanFilterBase<QNumberType, StateSize, MeasurementSize, ControlSize>::UpdateState(
        const MeasurementVector& innovation, const KalmanGain& K, const MeasurementMatrix& H)
    {
        state_ = state_ + K * innovation;

        auto IminusKH = StateMatrix::Identity() - K * H;
        covariance_ = IminusKH * covariance_ * IminusKH.Transpose() + K * measurementNoise_ * K.Transpose();
    }

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    const typename KalmanFilterBase<QNumberType, StateSize, MeasurementSize, ControlSize>::StateVector&
    KalmanFilterBase<QNumberType, StateSize, MeasurementSize, ControlSize>::GetState() const
    {
        return state_;
    }

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    const typename KalmanFilterBase<QNumberType, StateSize, MeasurementSize, ControlSize>::StateMatrix&
    KalmanFilterBase<QNumberType, StateSize, MeasurementSize, ControlSize>::GetCovariance() const
    {
        return covariance_;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class KalmanFilterBase<float, 2, 1, 0>;
    extern template class KalmanFilterBase<float, 2, 1, 1>;
    extern template class KalmanFilterBase<float, 3, 1, 0>;
    extern template class KalmanFilterBase<float, 4, 2, 0>;
#endif
}
