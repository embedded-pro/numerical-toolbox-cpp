#pragma once

#include "infra/util/Optional.hpp"
#include "numerical/controllers/interfaces/MpcController.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/solvers/GaussianElimination.hpp"
#include <algorithm>

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

namespace controllers
{
    template<typename T, std::size_t StateSize, std::size_t InputSize>
    struct MpcWeights
    {
        math::SquareMatrix<T, StateSize> Q;
        math::SquareMatrix<T, InputSize> R;
        infra::Optional<math::SquareMatrix<T, StateSize>> terminalP;
    };

    template<typename T, std::size_t InputSize>
    struct MpcConstraints
    {
        infra::Optional<math::Vector<T, InputSize>> uMin;
        infra::Optional<math::Vector<T, InputSize>> uMax;
    };

    namespace detail
    {
        template<typename T, std::size_t DestRows, std::size_t DestCols, std::size_t SrcRows, std::size_t SrcCols>
        ALWAYS_INLINE_HOT void SetBlock(math::Matrix<T, DestRows, DestCols>& dest,
            const math::Matrix<T, SrcRows, SrcCols>& src,
            std::size_t rowOffset, std::size_t colOffset)
        {
            for (std::size_t r = 0; r < SrcRows; ++r)
                for (std::size_t c = 0; c < SrcCols; ++c)
                    dest.at(rowOffset + r, colOffset + c) = src.at(r, c);
        }
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t PredictionHorizon, std::size_t ControlHorizon = PredictionHorizon>
    class Mpc
        : public MpcController<T, StateSize, InputSize, PredictionHorizon, ControlHorizon>
    {
        using Base = MpcController<T, StateSize, InputSize, PredictionHorizon, ControlHorizon>;

        static constexpr std::size_t TotalControlDim = ControlHorizon * InputSize;
        static constexpr std::size_t TotalStateDim = PredictionHorizon * StateSize;

    public:
        using typename Base::ControlSequence;
        using typename Base::InputMatrix;
        using typename Base::InputVector;
        using typename Base::StateMatrix;
        using typename Base::StateVector;
        using InputWeightMatrix = math::SquareMatrix<T, InputSize>;

        using HessianMatrix = math::SquareMatrix<T, TotalControlDim>;
        using GradientMatrix = math::Matrix<T, TotalControlDim, StateSize>;

        Mpc(const StateMatrix& A, const InputMatrix& B,
            const MpcWeights<T, StateSize, InputSize>& weights,
            const MpcConstraints<T, InputSize>& constraints = MpcConstraints<T, InputSize>{});

        Mpc(const HessianMatrix& precomputedH, const GradientMatrix& precomputedF,
            const MpcConstraints<T, InputSize>& constraints = MpcConstraints<T, InputSize>{});

        InputVector ComputeControl(const StateVector& state) override;
        [[nodiscard]] const ControlSequence& GetControlSequence() const override;

        void SetReference(const StateVector& reference);
        void ClearReference();

        [[nodiscard]] const HessianMatrix& GetHessian() const;
        [[nodiscard]] const GradientMatrix& GetGradientMatrix() const;

    private:
        using PredictionStateMatrix = math::Matrix<T, TotalStateDim, StateSize>;
        using PredictionInputMatrix = math::Matrix<T, TotalStateDim, TotalControlDim>;
        using AugmentedStateWeight = math::SquareMatrix<T, TotalStateDim>;
        using AugmentedInputWeight = math::SquareMatrix<T, TotalControlDim>;
        using ControlVector = math::Vector<T, TotalControlDim>;

        PredictionStateMatrix BuildPsi(const StateMatrix& A) const;
        PredictionInputMatrix BuildTheta(const StateMatrix& A, const InputMatrix& B) const;
        void BuildCostMatrices(const MpcWeights<T, StateSize, InputSize>& weights,
            const StateMatrix& A, const InputMatrix& B);
        void ApplyConstraints(ControlVector& u) const;

        HessianMatrix hessian;
        GradientMatrix gradientMatrix;
        MpcConstraints<T, InputSize> constraints;
        ControlSequence controlSequence;
        infra::Optional<StateVector> reference;
    };

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t PredictionHorizon, std::size_t ControlHorizon>
    Mpc<T, StateSize, InputSize, PredictionHorizon, ControlHorizon>::Mpc(
        const StateMatrix& A, const InputMatrix& B,
        const MpcWeights<T, StateSize, InputSize>& weights,
        const MpcConstraints<T, InputSize>& constraints)
        : constraints(constraints)
    {
        BuildCostMatrices(weights, A, B);
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t PredictionHorizon, std::size_t ControlHorizon>
    Mpc<T, StateSize, InputSize, PredictionHorizon, ControlHorizon>::Mpc(
        const HessianMatrix& precomputedH, const GradientMatrix& precomputedF,
        const MpcConstraints<T, InputSize>& constraints)
        : hessian(precomputedH)
        , gradientMatrix(precomputedF)
        , constraints(constraints)
    {}

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t PredictionHorizon, std::size_t ControlHorizon>
    OPTIMIZE_FOR_SPEED auto Mpc<T, StateSize, InputSize, PredictionHorizon, ControlHorizon>::BuildPsi(
        const StateMatrix& A) const -> PredictionStateMatrix
    {
        PredictionStateMatrix psi;
        auto Apow = StateMatrix::Identity();

        for (std::size_t k = 0; k < PredictionHorizon; ++k)
        {
            Apow = Apow * A;
            detail::SetBlock(psi, Apow, k * StateSize, 0);
        }

        return psi;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t PredictionHorizon, std::size_t ControlHorizon>
    OPTIMIZE_FOR_SPEED auto Mpc<T, StateSize, InputSize, PredictionHorizon, ControlHorizon>::BuildTheta(
        const StateMatrix& A, const InputMatrix& B) const -> PredictionInputMatrix
    {
        PredictionInputMatrix theta;

        for (std::size_t k = 0; k < PredictionHorizon; ++k)
        {
            for (std::size_t j = 0; j < ControlHorizon && j <= k; ++j)
            {
                auto ApowLocal = StateMatrix::Identity();
                for (std::size_t p = 0; p < k - j; ++p)
                    ApowLocal = ApowLocal * A;

                auto block = ApowLocal * B;
                detail::SetBlock(theta, block, k * StateSize, j * InputSize);
            }
        }

        return theta;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t PredictionHorizon, std::size_t ControlHorizon>
    OPTIMIZE_FOR_SPEED void Mpc<T, StateSize, InputSize, PredictionHorizon, ControlHorizon>::BuildCostMatrices(
        const MpcWeights<T, StateSize, InputSize>& weights,
        const StateMatrix& A, const InputMatrix& B)
    {
        auto psi = BuildPsi(A);
        auto theta = BuildTheta(A, B);

        AugmentedStateWeight qBar;
        for (std::size_t k = 0; k < PredictionHorizon; ++k)
        {
            const auto& Qk = (k == PredictionHorizon - 1 && weights.terminalP)
                                 ? *weights.terminalP
                                 : weights.Q;

            detail::SetBlock(qBar, Qk, k * StateSize, k * StateSize);
        }

        AugmentedInputWeight rBar;
        for (std::size_t k = 0; k < ControlHorizon; ++k)
            detail::SetBlock(rBar, weights.R, k * InputSize, k * InputSize);

        auto thetaT = theta.Transpose();
        hessian = thetaT * qBar * theta + rBar;
        gradientMatrix = thetaT * qBar * psi;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t PredictionHorizon, std::size_t ControlHorizon>
    OPTIMIZE_FOR_SPEED void Mpc<T, StateSize, InputSize, PredictionHorizon, ControlHorizon>::ApplyConstraints(ControlVector& u) const
    {
        if (!constraints.uMin && !constraints.uMax)
            return;

        for (std::size_t k = 0; k < ControlHorizon; ++k)
        {
            for (std::size_t i = 0; i < InputSize; ++i)
            {
                if (constraints.uMin)
                    u.at(k * InputSize + i, 0) = std::max(u.at(k * InputSize + i, 0), constraints.uMin->at(i, 0));
                if (constraints.uMax)
                    u.at(k * InputSize + i, 0) = std::min(u.at(k * InputSize + i, 0), constraints.uMax->at(i, 0));
            }
        }
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t PredictionHorizon, std::size_t ControlHorizon>
    OPTIMIZE_FOR_SPEED
        typename Mpc<T, StateSize, InputSize, PredictionHorizon, ControlHorizon>::InputVector
        Mpc<T, StateSize, InputSize, PredictionHorizon, ControlHorizon>::ComputeControl(const StateVector& state)
    {
        auto effectiveState = reference ? state - *reference : state;
        auto g = gradientMatrix * effectiveState;

        ControlVector negG;
        for (std::size_t i = 0; i < TotalControlDim; ++i)
            negG.at(i, 0) = T(-1.0f) * g.at(i, 0);

        auto uOptimal = solvers::SolveSystem<T, TotalControlDim, 1>(hessian, negG);
        ApplyConstraints(uOptimal);

        for (std::size_t k = 0; k < ControlHorizon; ++k)
            for (std::size_t i = 0; i < InputSize; ++i)
                controlSequence[k].at(i, 0) = uOptimal.at(k * InputSize + i, 0);

        return controlSequence[0];
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t PredictionHorizon, std::size_t ControlHorizon>
    const typename Mpc<T, StateSize, InputSize, PredictionHorizon, ControlHorizon>::ControlSequence&
    Mpc<T, StateSize, InputSize, PredictionHorizon, ControlHorizon>::GetControlSequence() const
    {
        return controlSequence;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t PredictionHorizon, std::size_t ControlHorizon>
    void Mpc<T, StateSize, InputSize, PredictionHorizon, ControlHorizon>::SetReference(const StateVector& ref)
    {
        reference = ref;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t PredictionHorizon, std::size_t ControlHorizon>
    void Mpc<T, StateSize, InputSize, PredictionHorizon, ControlHorizon>::ClearReference()
    {
        reference = infra::none;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t PredictionHorizon, std::size_t ControlHorizon>
    const typename Mpc<T, StateSize, InputSize, PredictionHorizon, ControlHorizon>::HessianMatrix&
    Mpc<T, StateSize, InputSize, PredictionHorizon, ControlHorizon>::GetHessian() const
    {
        return hessian;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t PredictionHorizon, std::size_t ControlHorizon>
    const typename Mpc<T, StateSize, InputSize, PredictionHorizon, ControlHorizon>::GradientMatrix&
    Mpc<T, StateSize, InputSize, PredictionHorizon, ControlHorizon>::GetGradientMatrix() const
    {
        return gradientMatrix;
    }

    extern template class Mpc<float, 2, 1, 5, 5>;
    extern template class Mpc<float, 2, 1, 10, 10>;
    extern template class Mpc<float, 3, 1, 10, 10>;
    extern template class Mpc<float, 2, 1, 10, 5>;
}
