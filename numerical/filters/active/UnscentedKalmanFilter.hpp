#pragma once

#include "infra/util/Function.hpp"
#include "numerical/filters/active/KalmanFilterBase.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/solvers/CholeskyDecomposition.hpp"
#include <array>
#include <cmath>

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

namespace filters
{
    struct UkfParameters
    {
        float alpha = 1e-3f;
        float beta = 2.0f;
        float kappa = 0.0f;
    };

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize = 0>
    class UnscentedKalmanFilter
        : public KalmanFilterBase<QNumberType, StateSize, MeasurementSize, ControlSize>
    {
        using Base = KalmanFilterBase<QNumberType, StateSize, MeasurementSize, ControlSize>;

        static constexpr std::size_t SigmaPointCount = 2 * StateSize + 1;

    public:
        using typename Base::ControlVector;
        using typename Base::MeasurementCovariance;
        using typename Base::MeasurementMatrix;
        using typename Base::MeasurementVector;
        using typename Base::StateMatrix;
        using typename Base::StateVector;

        using StateTransitionFn = infra::Function<StateVector(const StateVector&)>;
        using MeasurementFn = infra::Function<MeasurementVector(const StateVector&)>;

        using StateTransitionWithControlFn = infra::Function<StateVector(const StateVector&, const ControlVector&)>;

        template<std::size_t C = ControlSize, typename = std::enable_if_t<(C == 0)>>
        UnscentedKalmanFilter(const StateVector& initialState, const StateMatrix& initialCovariance,
            StateTransitionFn f, MeasurementFn h,
            UkfParameters params = UkfParameters{});

        template<std::size_t C = ControlSize, typename = std::enable_if_t<(C > 0)>>
        UnscentedKalmanFilter(const StateVector& initialState, const StateMatrix& initialCovariance,
            StateTransitionWithControlFn f, MeasurementFn h,
            UkfParameters params = UkfParameters{});

        template<std::size_t C = ControlSize, typename = std::enable_if_t<(C == 0)>>
        void Predict();

        template<std::size_t C = ControlSize, typename = std::enable_if_t<(C > 0)>>
        void Predict(const ControlVector& u);

        void Update(const MeasurementVector& measurement);

    private:
        void ComputeWeights();
        void GenerateSigmaPoints(std::array<StateVector, SigmaPointCount>& sigmaPoints) const;
        void PredictFromPropagatedSigmaPoints(const std::array<StateVector, SigmaPointCount>& sigmaPoints);

        template<std::size_t VectorSize>
        math::Vector<QNumberType, VectorSize> ComputeWeightedMean(
            const std::array<math::Vector<QNumberType, VectorSize>, SigmaPointCount>& points) const;

        template<std::size_t Size>
        math::SquareMatrix<QNumberType, Size> ComputeWeightedCovariance(
            const std::array<math::Vector<QNumberType, Size>, SigmaPointCount>& points,
            const math::Vector<QNumberType, Size>& mean) const;

        math::Matrix<QNumberType, StateSize, MeasurementSize> ComputeWeightedCrossCovariance(
            const std::array<StateVector, SigmaPointCount>& statePoints,
            const StateVector& stateMean,
            const std::array<MeasurementVector, SigmaPointCount>& measurementPoints,
            const MeasurementVector& measurementMean) const;

        StateTransitionFn stateTransitionFn;
        StateTransitionWithControlFn stateTransitionWithControlFn;
        MeasurementFn measurementFn;

        UkfParameters parameters;
        float lambda;
        std::array<float, SigmaPointCount> weightsMean;
        std::array<float, SigmaPointCount> weightsCovariance;
    };

    // Implementation //

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    template<std::size_t C, typename>
    UnscentedKalmanFilter<QNumberType, StateSize, MeasurementSize, ControlSize>::UnscentedKalmanFilter(
        const StateVector& initialState, const StateMatrix& initialCovariance,
        StateTransitionFn f, MeasurementFn h,
        UkfParameters params)
        : Base(initialState, initialCovariance)
        , stateTransitionFn(f)
        , measurementFn(h)
        , parameters(params)
    {
        ComputeWeights();
    }

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    template<std::size_t C, typename>
    UnscentedKalmanFilter<QNumberType, StateSize, MeasurementSize, ControlSize>::UnscentedKalmanFilter(
        const StateVector& initialState, const StateMatrix& initialCovariance,
        StateTransitionWithControlFn f, MeasurementFn h,
        UkfParameters params)
        : Base(initialState, initialCovariance)
        , stateTransitionWithControlFn(f)
        , measurementFn(h)
        , parameters(params)
    {
        ComputeWeights();
    }

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    void UnscentedKalmanFilter<QNumberType, StateSize, MeasurementSize, ControlSize>::ComputeWeights()
    {
        constexpr float n = static_cast<float>(StateSize);
        lambda = parameters.alpha * parameters.alpha * (n + parameters.kappa) - n;

        weightsMean[0] = lambda / (n + lambda);
        weightsCovariance[0] = lambda / (n + lambda) + (1.0f - parameters.alpha * parameters.alpha + parameters.beta);

        for (std::size_t i = 1; i < SigmaPointCount; ++i)
        {
            weightsMean[i] = 1.0f / (2.0f * (n + lambda));
            weightsCovariance[i] = 1.0f / (2.0f * (n + lambda));
        }
    }

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    void UnscentedKalmanFilter<QNumberType, StateSize, MeasurementSize, ControlSize>::GenerateSigmaPoints(
        std::array<StateVector, SigmaPointCount>& sigmaPoints) const
    {
        constexpr float n = static_cast<float>(StateSize);
        float scaleFactor = std::sqrt(n + lambda);
        auto L = solvers::CholeskyDecomposition<QNumberType, StateSize>(this->covariance);

        sigmaPoints[0] = this->state;

        // σᵢ = x̂ ± √(n+λ) · column_i(L)
        for (std::size_t i = 0; i < StateSize; ++i)
        {
            StateVector column;
            for (std::size_t j = 0; j < StateSize; ++j)
                column.at(j, 0) = QNumberType(math::ToFloat(L.at(j, i)) * scaleFactor);

            sigmaPoints[i + 1] = this->state + column;
            sigmaPoints[i + 1 + StateSize] = this->state - column;
        }
    }

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    template<std::size_t VectorSize>
    math::Vector<QNumberType, VectorSize>
    UnscentedKalmanFilter<QNumberType, StateSize, MeasurementSize, ControlSize>::ComputeWeightedMean(
        const std::array<math::Vector<QNumberType, VectorSize>, SigmaPointCount>& points) const
    {
        math::Vector<QNumberType, VectorSize> result;
        for (std::size_t i = 0; i < SigmaPointCount; ++i)
            for (std::size_t j = 0; j < VectorSize; ++j)
                result.at(j, 0) = QNumberType(
                    math::ToFloat(result.at(j, 0)) +
                    weightsMean[i] * math::ToFloat(points[i].at(j, 0)));
        return result;
    }

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    template<std::size_t Size>
    math::SquareMatrix<QNumberType, Size>
    UnscentedKalmanFilter<QNumberType, StateSize, MeasurementSize, ControlSize>::ComputeWeightedCovariance(
        const std::array<math::Vector<QNumberType, Size>, SigmaPointCount>& points,
        const math::Vector<QNumberType, Size>& mean) const
    {
        math::SquareMatrix<QNumberType, Size> result;
        for (std::size_t i = 0; i < SigmaPointCount; ++i)
        {
            auto diff = points[i] - mean;
            auto outer = diff * diff.Transpose();
            for (std::size_t r = 0; r < Size; ++r)
                for (std::size_t c = 0; c < Size; ++c)
                    result.at(r, c) = QNumberType(
                        math::ToFloat(result.at(r, c)) +
                        weightsCovariance[i] * math::ToFloat(outer.at(r, c)));
        }
        return result;
    }

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    math::Matrix<QNumberType, StateSize, MeasurementSize>
    UnscentedKalmanFilter<QNumberType, StateSize, MeasurementSize, ControlSize>::ComputeWeightedCrossCovariance(
        const std::array<StateVector, SigmaPointCount>& statePoints,
        const StateVector& stateMean,
        const std::array<MeasurementVector, SigmaPointCount>& measurementPoints,
        const MeasurementVector& measurementMean) const
    {
        math::Matrix<QNumberType, StateSize, MeasurementSize> result;
        for (std::size_t i = 0; i < SigmaPointCount; ++i)
        {
            auto stateDiff = statePoints[i] - stateMean;
            auto measDiff = measurementPoints[i] - measurementMean;
            auto outer = stateDiff * measDiff.Transpose();
            for (std::size_t r = 0; r < StateSize; ++r)
                for (std::size_t c = 0; c < MeasurementSize; ++c)
                    result.at(r, c) = QNumberType(
                        math::ToFloat(result.at(r, c)) +
                        weightsCovariance[i] * math::ToFloat(outer.at(r, c)));
        }
        return result;
    }

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    void UnscentedKalmanFilter<QNumberType, StateSize, MeasurementSize, ControlSize>::PredictFromPropagatedSigmaPoints(
        const std::array<StateVector, SigmaPointCount>& sigmaPoints)
    {
        this->state = ComputeWeightedMean<StateSize>(sigmaPoints);
        this->covariance = ComputeWeightedCovariance<StateSize>(sigmaPoints, this->state) + this->processNoise;
    }

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    template<std::size_t C, typename>
    OPTIMIZE_FOR_SPEED void UnscentedKalmanFilter<QNumberType, StateSize, MeasurementSize, ControlSize>::Predict()
    {
        std::array<StateVector, SigmaPointCount> sigmaPoints;
        GenerateSigmaPoints(sigmaPoints);

        for (std::size_t i = 0; i < SigmaPointCount; ++i)
            sigmaPoints[i] = stateTransitionFn(sigmaPoints[i]);

        PredictFromPropagatedSigmaPoints(sigmaPoints);
    }

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    template<std::size_t C, typename>
    OPTIMIZE_FOR_SPEED void UnscentedKalmanFilter<QNumberType, StateSize, MeasurementSize, ControlSize>::Predict(const ControlVector& u)
    {
        std::array<StateVector, SigmaPointCount> sigmaPoints;
        GenerateSigmaPoints(sigmaPoints);

        for (std::size_t i = 0; i < SigmaPointCount; ++i)
            sigmaPoints[i] = stateTransitionWithControlFn(sigmaPoints[i], u);

        PredictFromPropagatedSigmaPoints(sigmaPoints);
    }

    template<typename QNumberType, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    OPTIMIZE_FOR_SPEED void UnscentedKalmanFilter<QNumberType, StateSize, MeasurementSize, ControlSize>::Update(const MeasurementVector& measurement)
    {
        std::array<StateVector, SigmaPointCount> sigmaPoints;
        GenerateSigmaPoints(sigmaPoints);

        std::array<MeasurementVector, SigmaPointCount> measurementSigmaPoints;
        for (std::size_t i = 0; i < SigmaPointCount; ++i)
            measurementSigmaPoints[i] = measurementFn(sigmaPoints[i]);

        auto predictedMeasurement = ComputeWeightedMean<MeasurementSize>(measurementSigmaPoints);
        auto S = ComputeWeightedCovariance<MeasurementSize>(measurementSigmaPoints, predictedMeasurement) + this->measurementNoise;
        auto Pxz = ComputeWeightedCrossCovariance(sigmaPoints, this->state, measurementSigmaPoints, predictedMeasurement);

        // Solve K via Sᵀ Kᵀ = Pxzᵀ (avoids explicit S⁻¹)
        auto K = solvers::SolveSystem<QNumberType, MeasurementSize, StateSize>(S, Pxz.Transpose()).Transpose();

        auto innovation = measurement - predictedMeasurement;
        this->state = this->state + K * innovation;
        this->covariance = this->covariance - K * S * K.Transpose();
    }

    extern template class UnscentedKalmanFilter<float, 2, 1, 0>;
    extern template class UnscentedKalmanFilter<float, 2, 1, 1>;
    extern template class UnscentedKalmanFilter<float, 3, 1, 0>;
}
