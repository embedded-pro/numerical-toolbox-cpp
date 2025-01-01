#ifndef FILTERS_ACTIVE_KALMAN_FILTER_HPP
#define FILTERS_ACTIVE_KALMAN_FILTER_HPP

#include "math/Matrix.hpp"

namespace filters
{
    namespace detail
    {
        template<typename T>
        inline constexpr bool is_kalman_supported_v =
            std::disjunction_v<
                std::is_same<T, float>,
                std::is_same<T, math::Q15>,
                std::is_same<T, math::Q31>>;
    }

    template<typename T, std::size_t StateSize, std::size_t MeasurementSize>
    class KalmanFilter
    {
        static_assert(detail::is_kalman_supported_v<T>,
            "KalmanFilter only supports float, Q15, or Q31 types");
        static_assert(math::detail::is_valid_dimensions_v<StateSize, StateSize>,
            "State dimensions must be positive");
        static_assert(math::detail::is_valid_dimensions_v<MeasurementSize, MeasurementSize>,
            "Measurement dimensions must be positive");

    public:
        using StateMatrix = math::SquareMatrix<T, StateSize>;
        using StateVector = math::Vector<T, StateSize>;
        using MeasurementMatrix = math::Matrix<T, MeasurementSize, StateSize>;
        using MeasurementVector = math::Vector<T, MeasurementSize>;
        using MeasurementCovariance = math::SquareMatrix<T, MeasurementSize>;

        KalmanFilter(const StateVector& initialState, const StateMatrix& initialCovariance)
            : state(initialState)
            , covariance(initialCovariance)
            , stateTransition(StateMatrix::Identity())
            , processNoise(StateMatrix::Identity())
            , measurementMatrix(MeasurementMatrix())
            , measurementNoise(MeasurementCovariance::Identity())
        {}

        void SetStateTransition(const StateMatrix& F)
        {
            stateTransition = F;
        }

        void SetProcessNoise(const StateMatrix& Q)
        {
            processNoise = Q;
        }

        void SetMeasurementMatrix(const MeasurementMatrix& H)
        {
            measurementMatrix = H;
        }

        void SetMeasurementNoise(const MeasurementCovariance& R)
        {
            measurementNoise = R;
        }

        void Predict()
        {
            // State prediction: x̂ₖ₋ = Fₖ₋₁x̂ₖ₋₁
            state = stateTransition * state;

            // Covariance prediction: Pₖ₋ = Fₖ₋₁Pₖ₋₁Fₖ₋₁ᵀ + Qₖ₋₁
            covariance = stateTransition * covariance * stateTransition.Transpose() + processNoise;
        }

        void Update(const MeasurementVector& measurement)
        {
            // Innovation/measurement residual: yₖ = zₖ - Hₖx̂ₖ₋
            auto innovation = measurement - (measurementMatrix * state);

            // Innovation covariance: Sₖ = HₖPₖ₋Hₖᵀ + Rₖ
            auto innovationCovariance = measurementMatrix * covariance * measurementMatrix.Transpose() + measurementNoise;

            // Optimal Kalman gain: Kₖ = Pₖ₋Hₖᵀ(Sₖ)⁻¹
            // Note: For this implementation, we're assuming the inverse exists
            // In practice, you might want to add checks or use pseudo-inverse
            auto kalmanGain = covariance * measurementMatrix.Transpose() * innovationCovariance;

            // State update: x̂ₖ = x̂ₖ₋ + Kₖyₖ
            state = state + kalmanGain * innovation;

            // Covariance update: Pₖ = (I - KₖHₖ)Pₖ₋
            auto identity = StateMatrix::Identity();
            covariance = (identity - kalmanGain * measurementMatrix) * covariance;
        }

        [[nodiscard]] const StateVector& GetState() const
        {
            return state;
        }

        [[nodiscard]] const StateMatrix& GetCovariance() const
        {
            return covariance;
        }

    private:
        StateVector state;
        StateMatrix covariance;
        StateMatrix stateTransition;
        StateMatrix processNoise;
        MeasurementMatrix measurementMatrix;
        MeasurementCovariance measurementNoise;
    };
}

#endif
