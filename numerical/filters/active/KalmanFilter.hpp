#pragma once

#include "numerical/filters/active/KalmanFilterBase.hpp"
#include "numerical/math/CompilerOptimizations.hpp"

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

namespace filters
{
    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize = 0>
    class KalmanFilter
        : public KalmanFilterBase<QNumberType, StateSize, MeasurementSize, ControlSize>
    {
        using Base = KalmanFilterBase<QNumberType, StateSize, MeasurementSize, ControlSize>;

    public:
        using typename Base::ControlMatrix;
        using typename Base::ControlVector;
        using typename Base::MeasurementCovariance;
        using typename Base::MeasurementMatrix;
        using typename Base::MeasurementVector;
        using typename Base::StateMatrix;
        using typename Base::StateVector;

        KalmanFilter(const StateVector& initialState, const StateMatrix& initialCovariance);

        void SetStateTransition(const StateMatrix& F);
        void SetMeasurementMatrix(const MeasurementMatrix& H);

        template<std::size_t C = ControlSize, typename = std::enable_if_t<(C > 0)>>
        void SetControlInputMatrix(const ControlMatrix& B);

        void Predict();

        template<std::size_t C = ControlSize, typename = std::enable_if_t<(C > 0)>>
        void Predict(const ControlVector& u);

        void Update(const MeasurementVector& measurement);

    private:
        StateMatrix stateTransition;
        MeasurementMatrix measurementMatrix;
        [[no_unique_address]] ControlMatrix controlInputMatrix;
    };

    // Implementation //

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    KalmanFilter<QNumberType, StateSize, MeasurementSize, ControlSize>::KalmanFilter(
        const StateVector& initialState, const StateMatrix& initialCovariance)
        : Base(initialState, initialCovariance)
        , stateTransition(StateMatrix::Identity())
        , measurementMatrix(MeasurementMatrix())
        , controlInputMatrix(ControlMatrix())
    {}

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    void KalmanFilter<QNumberType, StateSize, MeasurementSize, ControlSize>::SetStateTransition(const StateMatrix& F)
    {
        stateTransition = F;
    }

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    void KalmanFilter<QNumberType, StateSize, MeasurementSize, ControlSize>::SetMeasurementMatrix(const MeasurementMatrix& H)
    {
        measurementMatrix = H;
    }

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    template<std::size_t C, typename>
    void KalmanFilter<QNumberType, StateSize, MeasurementSize, ControlSize>::SetControlInputMatrix(const ControlMatrix& B)
    {
        controlInputMatrix = B;
    }

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    OPTIMIZE_FOR_SPEED void KalmanFilter<QNumberType, StateSize, MeasurementSize, ControlSize>::Predict()
    {
        this->state = stateTransition * this->state;
        this->covariance = stateTransition * this->covariance * stateTransition.Transpose() + this->processNoise;
    }

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    template<std::size_t C, typename>
    OPTIMIZE_FOR_SPEED void KalmanFilter<QNumberType, StateSize, MeasurementSize, ControlSize>::Predict(const ControlVector& u)
    {
        this->state = stateTransition * this->state + controlInputMatrix * u;
        this->covariance = stateTransition * this->covariance * stateTransition.Transpose() + this->processNoise;
    }

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    OPTIMIZE_FOR_SPEED void KalmanFilter<QNumberType, StateSize, MeasurementSize, ControlSize>::Update(const MeasurementVector& measurement)
    {
        auto innovation = measurement - (measurementMatrix * this->state);
        auto K = this->ComputeKalmanGain(measurementMatrix);
        this->UpdateState(innovation, K, measurementMatrix);
    }

    extern template class KalmanFilter<float, 2, 1, 0>;
    extern template class KalmanFilter<float, 3, 1, 0>;
    extern template class KalmanFilter<float, 4, 2, 0>;
    extern template class KalmanFilter<float, 2, 1, 1>;
}
