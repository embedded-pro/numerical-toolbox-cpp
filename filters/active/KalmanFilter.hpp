#ifndef FILTERS_ACTIVE_KALMAN_FILTER_HPP
#define FILTERS_ACTIVE_KALMAN_FILTER_HPP

#include "math/Matrix.hpp"

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

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize>
    class KalmanFilter
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

        KalmanFilter(const StateVector& initialState, const StateMatrix& initialCovariance);

        void SetStateTransition(const StateMatrix& F);
        void SetProcessNoise(const StateMatrix& Q);
        void SetMeasurementMatrix(const MeasurementMatrix& H);
        void SetMeasurementNoise(const MeasurementCovariance& R);
        void Predict();
        void Update(const MeasurementVector& measurement);
        [[nodiscard]] const StateVector& GetState() const;
        [[nodiscard]] const StateMatrix& GetCovariance() const;

    private:
        StateVector state;
        StateMatrix covariance;
        StateMatrix stateTransition;
        StateMatrix processNoise;
        MeasurementMatrix measurementMatrix;
        MeasurementCovariance measurementNoise;
    };

    // Implementation //

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize>
    KalmanFilter<QNumberType, StateSize, MeasurementSize>::KalmanFilter(
        const StateVector& initialState, const StateMatrix& initialCovariance)
        : state(initialState)
        , covariance(initialCovariance)
        , stateTransition(StateMatrix::Identity())
        , processNoise(StateMatrix::Identity())
        , measurementMatrix(MeasurementMatrix())
        , measurementNoise(MeasurementCovariance::Identity())
    {}

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize>
    void KalmanFilter<QNumberType, StateSize, MeasurementSize>::SetStateTransition(const StateMatrix& F)
    {
        stateTransition = F;
    }

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize>
    void KalmanFilter<QNumberType, StateSize, MeasurementSize>::SetProcessNoise(const StateMatrix& Q)
    {
        processNoise = Q;
    }

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize>
    void KalmanFilter<QNumberType, StateSize, MeasurementSize>::SetMeasurementMatrix(const MeasurementMatrix& H)
    {
        measurementMatrix = H;
    }

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize>
    void KalmanFilter<QNumberType, StateSize, MeasurementSize>::SetMeasurementNoise(const MeasurementCovariance& R)
    {
        measurementNoise = R;
    }

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize>
    void KalmanFilter<QNumberType, StateSize, MeasurementSize>::Predict()
    {
        // State prediction: x̂ₖ₋ = Fₖ₋₁x̂ₖ₋₁
        state = stateTransition * state;

        // Covariance prediction: Pₖ₋ = Fₖ₋₁Pₖ₋₁Fₖ₋₁ᵀ + Qₖ₋₁
        covariance = stateTransition * covariance * stateTransition.Transpose() + processNoise;
    }

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize>
    void KalmanFilter<QNumberType, StateSize, MeasurementSize>::Update(const MeasurementVector& measurement)
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

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize>
    const typename KalmanFilter<QNumberType, StateSize, MeasurementSize>::StateVector&
    KalmanFilter<QNumberType, StateSize, MeasurementSize>::GetState() const
    {
        return state;
    }

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize>
    const typename KalmanFilter<QNumberType, StateSize, MeasurementSize>::StateMatrix&
    KalmanFilter<QNumberType, StateSize, MeasurementSize>::GetCovariance() const
    {
        return covariance;
    }
}

#endif
