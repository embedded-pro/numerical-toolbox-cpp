#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/controllers/implementations/Lqr.hpp"
#include "numerical/controllers/interfaces/OutputFeedbackController.hpp"
#include "numerical/filters/active/KalmanFilter.hpp"
#include "numerical/math/LinearTimeInvariant.hpp"

namespace controllers
{
    template<typename T, std::size_t StateSize, std::size_t InputSize>
    struct LqgWeights
    {
        math::SquareMatrix<T, StateSize> Q{};
        math::SquareMatrix<T, InputSize> R{};
    };

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t MeasurementSize>
    struct LqgNoise
    {
        math::SquareMatrix<T, StateSize> processNoise{};
        math::SquareMatrix<T, MeasurementSize> measurementNoise{};
    };

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t MeasurementSize>
    class Lqg
        : public OutputFeedbackController<T, StateSize, InputSize, MeasurementSize>
    {
        static_assert(StateSize > 0, "StateSize must be positive");
        static_assert(InputSize > 0, "InputSize must be positive");
        static_assert(MeasurementSize > 0, "MeasurementSize must be positive");

    public:
        using typename OutputFeedbackController<T, StateSize, InputSize, MeasurementSize>::InputVector;
        using typename OutputFeedbackController<T, StateSize, InputSize, MeasurementSize>::MeasurementVector;
        using StateVector = math::Vector<T, StateSize>;
        using StateCovarianceMatrix = math::SquareMatrix<T, StateSize>;

        Lqg(const math::LinearTimeInvariant<T, StateSize, InputSize, MeasurementSize>& plant,
            const LqgNoise<T, StateSize, InputSize, MeasurementSize>& noise,
            const LqgWeights<T, StateSize, InputSize>& weights,
            const StateVector& initialState,
            const StateCovarianceMatrix& initialCovariance);

        InputVector ComputeControl(const MeasurementVector& measurement) override;

        [[nodiscard]] StateVector GetEstimatedState() const;

        [[nodiscard]] const typename Lqr<T, StateSize, InputSize>::GainMatrix& GetGain() const;

    private:
        filters::KalmanFilter<T, StateSize, MeasurementSize, InputSize> estimator;
        Lqr<T, StateSize, InputSize> regulator;
    };

    // Implementation //

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t MeasurementSize>
    Lqg<T, StateSize, InputSize, MeasurementSize>::Lqg(
        const math::LinearTimeInvariant<T, StateSize, InputSize, MeasurementSize>& plant,
        const LqgNoise<T, StateSize, InputSize, MeasurementSize>& noise,
        const LqgWeights<T, StateSize, InputSize>& weights,
        const StateVector& initialState,
        const StateCovarianceMatrix& initialCovariance)
        : estimator{ initialState, initialCovariance }
        , regulator{ plant.A, plant.B, weights.Q, weights.R }
    {
        estimator.SetPlant(plant);
        estimator.SetProcessNoise(noise.processNoise);
        estimator.SetMeasurementNoise(noise.measurementNoise);
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t MeasurementSize>
    typename Lqg<T, StateSize, InputSize, MeasurementSize>::InputVector
    Lqg<T, StateSize, InputSize, MeasurementSize>::ComputeControl(const MeasurementVector& measurement)
    {
        estimator.Update(measurement);
        const auto u = regulator.ComputeControl(estimator.GetState());
        estimator.Predict(u);
        return u;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t MeasurementSize>
    typename Lqg<T, StateSize, InputSize, MeasurementSize>::StateVector
    Lqg<T, StateSize, InputSize, MeasurementSize>::GetEstimatedState() const
    {
        return estimator.GetState();
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t MeasurementSize>
    const typename Lqr<T, StateSize, InputSize>::GainMatrix&
    Lqg<T, StateSize, InputSize, MeasurementSize>::GetGain() const
    {
        return regulator.GetGain();
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class Lqg<float, 2, 1, 1>;
    extern template class Lqg<float, 4, 1, 1>;
#endif
}
