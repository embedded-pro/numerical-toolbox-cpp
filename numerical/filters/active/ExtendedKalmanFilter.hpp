#pragma once

#include "infra/util/Function.hpp"
#include "numerical/filters/active/KalmanFilterBase.hpp"
#include "numerical/math/CompilerOptimizations.hpp"

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

namespace filters
{
    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize = 0>
    class ExtendedKalmanFilter
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

        using StateTransitionFn = infra::Function<StateVector(const StateVector&)>;
        using StateJacobianFn = infra::Function<StateMatrix(const StateVector&)>;
        using MeasurementFn = infra::Function<MeasurementVector(const StateVector&)>;
        using MeasurementJacobianFn = infra::Function<MeasurementMatrix(const StateVector&)>;

        using StateTransitionWithControlFn = infra::Function<StateVector(const StateVector&, const ControlVector&)>;
        using StateJacobianWithControlFn = infra::Function<StateMatrix(const StateVector&, const ControlVector&)>;

        template<std::size_t C = ControlSize, typename = std::enable_if_t<(C == 0)>>
        ExtendedKalmanFilter(const StateVector& initialState, const StateMatrix& initialCovariance,
            StateTransitionFn f, StateJacobianFn jf,
            MeasurementFn h, MeasurementJacobianFn jh);

        template<std::size_t C = ControlSize, typename = std::enable_if_t<(C > 0)>>
        ExtendedKalmanFilter(const StateVector& initialState, const StateMatrix& initialCovariance,
            StateTransitionWithControlFn f, StateJacobianWithControlFn jf,
            MeasurementFn h, MeasurementJacobianFn jh);

        template<std::size_t C = ControlSize, typename = std::enable_if_t<(C == 0)>>
        void Predict();

        template<std::size_t C = ControlSize, typename = std::enable_if_t<(C > 0)>>
        void Predict(const ControlVector& u);

        void Update(const MeasurementVector& measurement);

    private:
        StateTransitionFn stateTransitionFn;
        StateJacobianFn stateJacobianFn;
        StateTransitionWithControlFn stateTransitionWithControlFn;
        StateJacobianWithControlFn stateJacobianWithControlFn;
        MeasurementFn measurementFn;
        MeasurementJacobianFn measurementJacobianFn;
    };

    // Implementation //

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    template<std::size_t C, typename>
    ExtendedKalmanFilter<QNumberType, StateSize, MeasurementSize, ControlSize>::ExtendedKalmanFilter(
        const StateVector& initialState, const StateMatrix& initialCovariance,
        StateTransitionFn f, StateJacobianFn jf,
        MeasurementFn h, MeasurementJacobianFn jh)
        : Base(initialState, initialCovariance)
        , stateTransitionFn(f)
        , stateJacobianFn(jf)
        , measurementFn(h)
        , measurementJacobianFn(jh)
    {}

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    template<std::size_t C, typename>
    ExtendedKalmanFilter<QNumberType, StateSize, MeasurementSize, ControlSize>::ExtendedKalmanFilter(
        const StateVector& initialState, const StateMatrix& initialCovariance,
        StateTransitionWithControlFn f, StateJacobianWithControlFn jf,
        MeasurementFn h, MeasurementJacobianFn jh)
        : Base(initialState, initialCovariance)
        , stateTransitionWithControlFn(f)
        , stateJacobianWithControlFn(jf)
        , measurementFn(h)
        , measurementJacobianFn(jh)
    {}

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    template<std::size_t C, typename>
    OPTIMIZE_FOR_SPEED void ExtendedKalmanFilter<QNumberType, StateSize, MeasurementSize, ControlSize>::Predict()
    {
        auto F = stateJacobianFn(this->state);
        this->state = stateTransitionFn(this->state);
        this->covariance = F * this->covariance * F.Transpose() + this->processNoise;
    }

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    template<std::size_t C, typename>
    OPTIMIZE_FOR_SPEED void ExtendedKalmanFilter<QNumberType, StateSize, MeasurementSize, ControlSize>::Predict(const ControlVector& u)
    {
        auto F = stateJacobianWithControlFn(this->state, u);
        this->state = stateTransitionWithControlFn(this->state, u);
        this->covariance = F * this->covariance * F.Transpose() + this->processNoise;
    }

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    OPTIMIZE_FOR_SPEED void ExtendedKalmanFilter<QNumberType, StateSize, MeasurementSize, ControlSize>::Update(const MeasurementVector& measurement)
    {
        auto H = measurementJacobianFn(this->state);
        auto innovation = measurement - measurementFn(this->state);
        auto K = this->ComputeKalmanGain(H);
        this->UpdateState(innovation, K, H);
    }

    extern template class ExtendedKalmanFilter<float, 2, 1, 0>;
    extern template class ExtendedKalmanFilter<float, 2, 1, 1>;
    extern template class ExtendedKalmanFilter<float, 3, 1, 0>;
}
