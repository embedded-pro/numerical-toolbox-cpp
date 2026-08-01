#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/filters/active/KalmanFilterBase.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/GivensRotation.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/TriangularSolve.hpp"
#include <array>
#include <cmath>
#include <cstddef>
#include <type_traits>

namespace filters
{
    template<typename T, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize = 0>
    class SquareRootKalmanFilter
    {
        static_assert(std::is_floating_point_v<T>, "SquareRootKalmanFilter supports floating-point types");
        static_assert(StateSize > 0, "StateSize must be positive");
        static_assert(MeasurementSize > 0, "MeasurementSize must be positive");

    public:
        using StateMatrix = math::SquareMatrix<T, StateSize>;
        using StateVector = math::Vector<T, StateSize>;
        using MeasurementMatrix = math::Matrix<T, MeasurementSize, StateSize>;
        using MeasurementVector = math::Vector<T, MeasurementSize>;
        using MeasurementCovariance = math::SquareMatrix<T, MeasurementSize>;
        using KalmanGain = math::Matrix<T, StateSize, MeasurementSize>;
        using ControlMatrix = typename detail::ControlType<T, StateSize, ControlSize>::Matrix;
        using ControlVector = typename detail::ControlType<T, StateSize, ControlSize>::Vector;

        SquareRootKalmanFilter(const StateVector& initialState, const StateMatrix& initialFactor);

        void SetStateTransition(const StateMatrix& F);
        void SetMeasurementMatrix(const MeasurementMatrix& H);
        void SetProcessNoiseFactor(const StateMatrix& sqrtQ);
        void SetMeasurementNoiseFactor(const MeasurementCovariance& sqrtR);

        void SetControlInputMatrix(const ControlMatrix& B)
        requires(ControlSize > 0);

        OPTIMIZE_FOR_SPEED void Predict()
        requires(ControlSize == 0);

        OPTIMIZE_FOR_SPEED void Predict(const ControlVector& u)
        requires(ControlSize > 0);

        OPTIMIZE_FOR_SPEED void Update(const MeasurementVector& z);

        [[nodiscard]] const StateVector& GetState() const;
        [[nodiscard]] StateMatrix GetCovariance() const;
        [[nodiscard]] const StateMatrix& GetCovarianceFactor() const;

    private:
        static constexpr std::size_t PredictRows = 2 * StateSize;
        static constexpr std::size_t UpdatePreRows = MeasurementSize + StateSize;

        using PredictArray = math::Matrix<T, PredictRows, StateSize>;
        using UpdateArray = math::Matrix<T, UpdatePreRows, UpdatePreRows>;

        OPTIMIZE_FOR_SPEED void PredictInternal();

        template<std::size_t Rows, std::size_t Cols>
        OPTIMIZE_FOR_SPEED static void QrTriangularize(math::Matrix<T, Rows, Cols>& A);

        [[nodiscard]] OPTIMIZE_FOR_SPEED static StateMatrix LowerFactor(const StateMatrix& upper);

        StateVector state_;
        StateMatrix factor_;
        StateMatrix stateTransition_{ StateMatrix::Identity() };
        MeasurementMatrix measurementMatrix_{};
        StateMatrix sqrtQ_{ StateMatrix::Identity() };
        MeasurementCovariance sqrtR_{ MeasurementCovariance::Identity() };
        [[no_unique_address]] ControlMatrix controlInputMatrix_{};
    };

    template<typename T, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    SquareRootKalmanFilter<T, StateSize, MeasurementSize, ControlSize>::SquareRootKalmanFilter(
        const StateVector& initialState, const StateMatrix& initialFactor)
        : state_(initialState)
        , factor_(initialFactor)
    {}

    template<typename T, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    void SquareRootKalmanFilter<T, StateSize, MeasurementSize, ControlSize>::SetStateTransition(const StateMatrix& F)
    {
        stateTransition_ = F;
    }

    template<typename T, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    void SquareRootKalmanFilter<T, StateSize, MeasurementSize, ControlSize>::SetMeasurementMatrix(const MeasurementMatrix& H)
    {
        measurementMatrix_ = H;
    }

    template<typename T, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    void SquareRootKalmanFilter<T, StateSize, MeasurementSize, ControlSize>::SetProcessNoiseFactor(const StateMatrix& sqrtQ)
    {
        sqrtQ_ = sqrtQ;
    }

    template<typename T, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    void SquareRootKalmanFilter<T, StateSize, MeasurementSize, ControlSize>::SetMeasurementNoiseFactor(const MeasurementCovariance& sqrtR)
    {
        sqrtR_ = sqrtR;
    }

    template<typename T, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    void SquareRootKalmanFilter<T, StateSize, MeasurementSize, ControlSize>::SetControlInputMatrix(const ControlMatrix& B)
    requires(ControlSize > 0)
    {
        controlInputMatrix_ = B;
    }

    template<typename T, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    template<std::size_t Rows, std::size_t Cols>
    OPTIMIZE_FOR_SPEED void SquareRootKalmanFilter<T, StateSize, MeasurementSize, ControlSize>::QrTriangularize(
        math::Matrix<T, Rows, Cols>& A)
    {
        for (std::size_t col = 0; col < Cols; ++col)
        {
            for (std::size_t row = Rows - 1; row > col; --row)
            {
                auto g = math::ComputeGivens(A.at(row - 1, col), A.at(row, col));
                for (std::size_t k = 0; k < Cols; ++k)
                    math::ApplyGivens(g, A.at(row - 1, k), A.at(row, k));
            }
        }
    }

    template<typename T, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    OPTIMIZE_FOR_SPEED typename SquareRootKalmanFilter<T, StateSize, MeasurementSize, ControlSize>::StateMatrix
    SquareRootKalmanFilter<T, StateSize, MeasurementSize, ControlSize>::LowerFactor(const StateMatrix& upper)
    {
        auto lower = upper.Transpose();

        for (std::size_t i = 0; i < StateSize; ++i)
            for (std::size_t j = i + 1; j < StateSize; ++j)
                lower.at(i, j) = T{};

        return lower;
    }

    template<typename T, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    OPTIMIZE_FOR_SPEED void SquareRootKalmanFilter<T, StateSize, MeasurementSize, ControlSize>::PredictInternal()
    {
        PredictArray A{};
        A.SetBlock((stateTransition_ * factor_).Transpose(), 0, 0);
        A.SetBlock(sqrtQ_.Transpose(), StateSize, 0);

        QrTriangularize(A);

        factor_ = LowerFactor(A.template GetBlock<StateSize, StateSize>(0, 0));
    }

    template<typename T, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    OPTIMIZE_FOR_SPEED void SquareRootKalmanFilter<T, StateSize, MeasurementSize, ControlSize>::Predict()
    requires(ControlSize == 0)
    {
        state_ = stateTransition_ * state_;
        PredictInternal();
    }

    template<typename T, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    OPTIMIZE_FOR_SPEED void SquareRootKalmanFilter<T, StateSize, MeasurementSize, ControlSize>::Predict(const ControlVector& u)
    requires(ControlSize > 0)
    {
        state_ = stateTransition_ * state_ + controlInputMatrix_ * u;
        PredictInternal();
    }

    template<typename T, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    OPTIMIZE_FOR_SPEED void SquareRootKalmanFilter<T, StateSize, MeasurementSize, ControlSize>::Update(const MeasurementVector& z)
    {
        UpdateArray A{};
        A.SetBlock(sqrtR_.Transpose(), 0, 0);
        A.SetBlock((measurementMatrix_ * factor_).Transpose(), MeasurementSize, 0);
        A.SetBlock(factor_.Transpose(), MeasurementSize, MeasurementSize);

        QrTriangularize(A);

        auto Sy = A.template GetBlock<MeasurementSize, MeasurementSize>(0, 0);
        auto cross = A.template GetBlock<MeasurementSize, StateSize>(0, MeasurementSize);

        KalmanGain K{};
        for (std::size_t j = 0; j < StateSize; ++j)
            K.SetBlock(math::SolveUpperTriangular(Sy, cross.GetColumn(j)).Transpose(), j, 0);

        auto innov = z - measurementMatrix_ * state_;
        state_ = state_ + K * innov;

        factor_ = LowerFactor(A.template GetBlock<StateSize, StateSize>(MeasurementSize, MeasurementSize));
    }

    template<typename T, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    const typename SquareRootKalmanFilter<T, StateSize, MeasurementSize, ControlSize>::StateVector&
    SquareRootKalmanFilter<T, StateSize, MeasurementSize, ControlSize>::GetState() const
    {
        return state_;
    }

    template<typename T, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    typename SquareRootKalmanFilter<T, StateSize, MeasurementSize, ControlSize>::StateMatrix
    SquareRootKalmanFilter<T, StateSize, MeasurementSize, ControlSize>::GetCovariance() const
    {
        return factor_ * factor_.Transpose();
    }

    template<typename T, std::size_t StateSize, std::size_t MeasurementSize, std::size_t ControlSize>
    const typename SquareRootKalmanFilter<T, StateSize, MeasurementSize, ControlSize>::StateMatrix&
    SquareRootKalmanFilter<T, StateSize, MeasurementSize, ControlSize>::GetCovarianceFactor() const
    {
        return factor_;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class SquareRootKalmanFilter<float, 2, 1, 0>;
    extern template class SquareRootKalmanFilter<float, 2, 1, 1>;
    extern template class SquareRootKalmanFilter<float, 3, 1, 0>;
#endif
}
