#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CholeskyDecomposition.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/solvers/DiscreteAlgebraicRiccatiEquation.hpp"
#include "numerical/solvers/GaussianElimination.hpp"
#include "numerical/solvers/SpectralRadius.hpp"
#include <algorithm>
#include <cstddef>
#include <limits>
#include <optional>
#include <type_traits>

namespace robust_control
{
    template<typename T,
        std::size_t StateSize,
        std::size_t DisturbanceSize,
        std::size_t ControlSize,
        std::size_t ErrorSize>
    struct GeneralizedPlant
    {
        static_assert(std::is_floating_point_v<T>, "GeneralizedPlant supports floating-point types");

        using StateMatrix = math::SquareMatrix<T, StateSize>;
        using DisturbanceMatrix = math::Matrix<T, StateSize, DisturbanceSize>;
        using ControlMatrix = math::Matrix<T, StateSize, ControlSize>;
        using ErrorStateMatrix = math::Matrix<T, ErrorSize, StateSize>;
        using ErrorControlMatrix = math::Matrix<T, ErrorSize, ControlSize>;

        StateMatrix A{};
        DisturbanceMatrix B1{};
        ControlMatrix B2{};
        ErrorStateMatrix C1{};
        ErrorControlMatrix D12{};
    };

    template<typename T,
        std::size_t StateSize,
        std::size_t DisturbanceSize,
        std::size_t ControlSize,
        std::size_t ErrorSize>
    class HInfinityStateFeedback
    {
        static_assert(std::is_floating_point_v<T>, "HInfinityStateFeedback supports floating-point types");

        static constexpr std::size_t AugInputSize = ControlSize + DisturbanceSize;

    public:
        using StateVector = math::Vector<T, StateSize>;
        using InputVector = math::Vector<T, ControlSize>;
        using GainMatrix = math::Matrix<T, ControlSize, StateSize>;
        using RiccatiMatrix = math::SquareMatrix<T, StateSize>;
        using Plant = GeneralizedPlant<T, StateSize, DisturbanceSize, ControlSize, ErrorSize>;

        explicit HInfinityStateFeedback(const Plant& gplant);

        bool Synthesize(T gammaMin, T gammaMax, T tol);

        OPTIMIZE_FOR_SPEED InputVector ComputeControl(const StateVector& x) const;

        [[nodiscard]] const GainMatrix& Gain() const;
        [[nodiscard]] T Gamma() const;

    private:
        using AugInputMatrix = math::Matrix<T, StateSize, AugInputSize>;
        using AugWeightMatrix = math::SquareMatrix<T, AugInputSize>;
        using AugGainMatrix = math::Matrix<T, AugInputSize, StateSize>;

        using CrossMatrix = math::Matrix<T, StateSize, AugInputSize>;

        struct Augmented
        {
            AugInputMatrix B{};
            AugWeightMatrix Rtilde{};
            RiccatiMatrix Q{};
            CrossMatrix N{};
            math::SquareMatrix<T, StateSize> Aeff{};
            RiccatiMatrix Qeff{};
        };

        std::optional<Augmented> BuildAugmented(T g) const;
        std::optional<AugGainMatrix> FullGain(const Augmented& aug, const RiccatiMatrix& Xsol) const;
        static bool IsPositiveDefinite(const math::SquareMatrix<T, StateSize>& m);
        bool RiccatiFeasible(T g) const;
        void SolveGameRiccati(T g);

        Plant plant{};
        GainMatrix K{};
        RiccatiMatrix X{};
        T gamma{};
    };

    template<typename T, std::size_t StateSize, std::size_t DisturbanceSize, std::size_t ControlSize, std::size_t ErrorSize>
    HInfinityStateFeedback<T, StateSize, DisturbanceSize, ControlSize, ErrorSize>::HInfinityStateFeedback(
        const Plant& gplant)
        : plant{ gplant }
    {}

    template<typename T, std::size_t StateSize, std::size_t DisturbanceSize, std::size_t ControlSize, std::size_t ErrorSize>
    std::optional<typename HInfinityStateFeedback<T, StateSize, DisturbanceSize, ControlSize, ErrorSize>::Augmented>
    HInfinityStateFeedback<T, StateSize, DisturbanceSize, ControlSize, ErrorSize>::BuildAugmented(T g) const
    {
        Augmented aug{};
        aug.B.SetBlock(plant.B2, 0, 0);
        aug.B.SetBlock(plant.B1, 0, ControlSize);

        const auto controlWeight = plant.D12.Transpose() * plant.D12;
        auto disturbanceWeight = math::SquareMatrix<T, DisturbanceSize>::Identity();
        disturbanceWeight *= -(g * g);
        aug.Rtilde.SetBlock(controlWeight, 0, 0);
        aug.Rtilde.SetBlock(disturbanceWeight, ControlSize, ControlSize);

        aug.Q = plant.C1.Transpose() * plant.C1;
        aug.N.SetBlock(plant.C1.Transpose() * plant.D12, 0, 0);

        const auto rInverse = solvers::TrySolveSystem<T, AugInputSize, AugInputSize>(
            aug.Rtilde, math::SquareMatrix<T, AugInputSize>::Identity());

        if (!rInverse)
            return std::nullopt;

        aug.Aeff = plant.A - aug.B * (*rInverse) * aug.N.Transpose();
        aug.Qeff = aug.Q - aug.N * (*rInverse) * aug.N.Transpose();

        return aug;
    }

    template<typename T, std::size_t StateSize, std::size_t DisturbanceSize, std::size_t ControlSize, std::size_t ErrorSize>
    bool HInfinityStateFeedback<T, StateSize, DisturbanceSize, ControlSize, ErrorSize>::IsPositiveDefinite(
        const math::SquareMatrix<T, StateSize>& m)
    {
        return math::CholeskyDecomposition<T, StateSize>::TryFactor(m).has_value();
    }

    template<typename T, std::size_t StateSize, std::size_t DisturbanceSize, std::size_t ControlSize, std::size_t ErrorSize>
    std::optional<typename HInfinityStateFeedback<T, StateSize, DisturbanceSize, ControlSize, ErrorSize>::AugGainMatrix>
    HInfinityStateFeedback<T, StateSize, DisturbanceSize, ControlSize, ErrorSize>::FullGain(
        const Augmented& aug, const RiccatiMatrix& Xsol) const
    {
        auto BtX = aug.B.Transpose() * Xsol;
        auto S = aug.Rtilde + BtX * aug.B;
        return solvers::TrySolveSystem<T, AugInputSize, StateSize>(S, BtX * plant.A + aug.N.Transpose());
    }

    template<typename T, std::size_t StateSize, std::size_t DisturbanceSize, std::size_t ControlSize, std::size_t ErrorSize>
    bool HInfinityStateFeedback<T, StateSize, DisturbanceSize, ControlSize, ErrorSize>::RiccatiFeasible(T g) const
    {
        const auto augmented = BuildAugmented(g);

        if (!augmented)
            return false;

        const Augmented& aug = *augmented;

        solvers::DiscreteAlgebraicRiccatiEquation<T, StateSize, AugInputSize> dare{};
        auto xcResult = dare.Solve(aug.Aeff, aug.B, aug.Qeff, aug.Rtilde);
        if (!xcResult.converged)
            return false;
        auto Xcandidate = xcResult.value;

        if (!IsPositiveDefinite(Xcandidate))
            return false;

        auto disturbanceMargin = math::SquareMatrix<T, DisturbanceSize>::Identity();
        disturbanceMargin *= g * g;
        disturbanceMargin = disturbanceMargin - plant.B1.Transpose() * (Xcandidate * plant.B1);

        if (!math::CholeskyDecomposition<T, DisturbanceSize>::TryFactor(disturbanceMargin).has_value())
            return false;

        const auto gain = FullGain(aug, Xcandidate);

        if (!gain)
            return false;

        auto AtX = plant.A.Transpose() * Xcandidate;
        auto Xresid = AtX * plant.A - (AtX * aug.B + aug.N) * (*gain) + aug.Q;

        T scale{ T{ 1 } };
        for (std::size_t i = 0; i < StateSize; ++i)
            for (std::size_t j = 0; j < StateSize; ++j)
                scale = std::max(scale, math::Abs(Xcandidate.at(i, j)));

        const T residualTolerance{ scale * std::numeric_limits<T>::epsilon() * static_cast<T>(StateSize * StateSize) * T{ 64 } };

        for (std::size_t i = 0; i < StateSize; ++i)
            for (std::size_t j = 0; j < StateSize; ++j)
                if (math::Abs(Xresid.at(i, j) - Xcandidate.at(i, j)) > residualTolerance)
                    return false;

        return true;
    }

    template<typename T, std::size_t StateSize, std::size_t DisturbanceSize, std::size_t ControlSize, std::size_t ErrorSize>
    void HInfinityStateFeedback<T, StateSize, DisturbanceSize, ControlSize, ErrorSize>::SolveGameRiccati(T g)
    {
        const auto augmented = BuildAugmented(g);

        if (!augmented)
            return;

        const Augmented& aug = *augmented;

        solvers::DiscreteAlgebraicRiccatiEquation<T, StateSize, AugInputSize> dare{};
        auto xResult = dare.Solve(aug.Aeff, aug.B, aug.Qeff, aug.Rtilde);
        if (!xResult.converged)
            return;
        X = xResult.value;

        const auto gain = FullGain(aug, X);

        if (!gain)
            return;

        K = gain->template GetBlock<ControlSize, StateSize>(0, 0);
    }

    template<typename T, std::size_t StateSize, std::size_t DisturbanceSize, std::size_t ControlSize, std::size_t ErrorSize>
    bool HInfinityStateFeedback<T, StateSize, DisturbanceSize, ControlSize, ErrorSize>::Synthesize(
        T gammaMin, T gammaMax, T tol)
    {
        T gLo{ gammaMin };
        T gHi{ gammaMax };
        bool anyFeasible{ false };

        while (gHi - gLo > tol)
        {
            const T gMid = (gLo + gHi) / T{ 2 };
            if (RiccatiFeasible(gMid))
            {
                gHi = gMid;
                anyFeasible = true;
            }
            else
                gLo = gMid;
        }

        if (!anyFeasible && !RiccatiFeasible(gHi))
            return false;

        gamma = gHi;
        SolveGameRiccati(gamma);

        auto closedLoop = plant.A - plant.B2 * K;
        return solvers::SpectralRadius<T, StateSize>{}.IsSchurStable(closedLoop);
    }

    template<typename T, std::size_t StateSize, std::size_t DisturbanceSize, std::size_t ControlSize, std::size_t ErrorSize>
    OPTIMIZE_FOR_SPEED
        typename HInfinityStateFeedback<T, StateSize, DisturbanceSize, ControlSize, ErrorSize>::InputVector
        HInfinityStateFeedback<T, StateSize, DisturbanceSize, ControlSize, ErrorSize>::ComputeControl(
            const StateVector& x) const
    {
        return K * x * T{ -1 };
    }

    template<typename T, std::size_t StateSize, std::size_t DisturbanceSize, std::size_t ControlSize, std::size_t ErrorSize>
    const typename HInfinityStateFeedback<T, StateSize, DisturbanceSize, ControlSize, ErrorSize>::GainMatrix&
    HInfinityStateFeedback<T, StateSize, DisturbanceSize, ControlSize, ErrorSize>::Gain() const
    {
        return K;
    }

    template<typename T, std::size_t StateSize, std::size_t DisturbanceSize, std::size_t ControlSize, std::size_t ErrorSize>
    T HInfinityStateFeedback<T, StateSize, DisturbanceSize, ControlSize, ErrorSize>::Gamma() const
    {
        return gamma;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class HInfinityStateFeedback<float, 2, 1, 1, 1>;
#endif
}
