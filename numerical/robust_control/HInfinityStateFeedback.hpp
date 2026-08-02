#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/LinearTimeInvariant.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/Tolerance.hpp"
#include "numerical/solvers/DiscreteAlgebraicRiccatiEquation.hpp"
#include "numerical/solvers/DurandKerner.hpp"
#include "numerical/solvers/GaussianElimination.hpp"
#include <array>
#include <cmath>
#include <cstddef>
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

        bool RiccatiFeasible(T g) const;
        void SolveGameRiccati(T g);
        bool IsSchurStable(const RiccatiMatrix& clA) const;

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
    bool HInfinityStateFeedback<T, StateSize, DisturbanceSize, ControlSize, ErrorSize>::RiccatiFeasible(T g) const
    {
        AugInputMatrix B{};
        for (std::size_t r = 0; r < StateSize; ++r)
        {
            for (std::size_t c = 0; c < ControlSize; ++c)
                B.at(r, c) = plant.B2.at(r, c);
            for (std::size_t c = 0; c < DisturbanceSize; ++c)
                B.at(r, ControlSize + c) = plant.B1.at(r, c);
        }

        AugWeightMatrix Rtilde{};
        for (std::size_t i = 0; i < ControlSize; ++i)
            Rtilde.at(i, i) = T{ 1 };
        const T negGammaSq = -(g * g);
        for (std::size_t i = 0; i < DisturbanceSize; ++i)
            Rtilde.at(ControlSize + i, ControlSize + i) = negGammaSq;

        auto Q = plant.C1.Transpose() * plant.C1;

        solvers::DiscreteAlgebraicRiccatiEquation<T, StateSize, AugInputSize> dare{};
        auto Xcandidate = dare.Solve(plant.A, B, Q, Rtilde);

        for (std::size_t i = 0; i < StateSize; ++i)
            if (Xcandidate.at(i, i) < T{ 0 })
                return false;

        auto distBlock = plant.B1.Transpose() * (Xcandidate * plant.B1);
        const T gammaSq = g * g;
        for (std::size_t i = 0; i < DisturbanceSize; ++i)
        {
            if (distBlock.at(i, i) >= gammaSq)
                return false;
        }

        auto BtX = B.Transpose() * Xcandidate;
        auto S = Rtilde + BtX * B;
        auto AtX = plant.A.Transpose() * Xcandidate;
        auto Xresid = AtX * plant.A - AtX * B * solvers::SolveSystem<T, AugInputSize, StateSize>(S, BtX * plant.A) + Q;

        for (std::size_t i = 0; i < StateSize; ++i)
            for (std::size_t j = 0; j < StateSize; ++j)
            {
                const T diff = std::abs(Xresid.at(i, j) - Xcandidate.at(i, j));
                if (diff > T{ 1 })
                    return false;
            }

        return true;
    }

    template<typename T, std::size_t StateSize, std::size_t DisturbanceSize, std::size_t ControlSize, std::size_t ErrorSize>
    void HInfinityStateFeedback<T, StateSize, DisturbanceSize, ControlSize, ErrorSize>::SolveGameRiccati(T g)
    {
        AugInputMatrix B{};
        for (std::size_t r = 0; r < StateSize; ++r)
        {
            for (std::size_t c = 0; c < ControlSize; ++c)
                B.at(r, c) = plant.B2.at(r, c);
            for (std::size_t c = 0; c < DisturbanceSize; ++c)
                B.at(r, ControlSize + c) = plant.B1.at(r, c);
        }

        AugWeightMatrix Rtilde{};
        for (std::size_t i = 0; i < ControlSize; ++i)
            Rtilde.at(i, i) = T{ 1 };
        for (std::size_t i = 0; i < DisturbanceSize; ++i)
            Rtilde.at(ControlSize + i, ControlSize + i) = -(g * g);

        auto Q = plant.C1.Transpose() * plant.C1;

        solvers::DiscreteAlgebraicRiccatiEquation<T, StateSize, AugInputSize> dare{};
        X = dare.Solve(plant.A, B, Q, Rtilde);

        auto BtX = B.Transpose() * X;
        auto S = Rtilde + BtX * B;
        auto BtXA = BtX * plant.A;
        auto Kfull = solvers::SolveSystem<T, AugInputSize, StateSize>(S, BtXA);

        for (std::size_t r = 0; r < ControlSize; ++r)
            for (std::size_t c = 0; c < StateSize; ++c)
                K.at(r, c) = Kfull.at(r, c);
    }

    template<typename T, std::size_t StateSize, std::size_t DisturbanceSize, std::size_t ControlSize, std::size_t ErrorSize>
    bool HInfinityStateFeedback<T, StateSize, DisturbanceSize, ControlSize, ErrorSize>::IsSchurStable(
        const RiccatiMatrix& clA) const
    {
        std::array<T, StateSize + 1> charPoly{};
        charPoly[0] = T{ 1 };

        math::SquareMatrix<T, StateSize> M{ clA };
        math::SquareMatrix<T, StateSize> prev{};

        for (std::size_t k = 1; k <= StateSize; ++k)
        {
            T trace{};
            for (std::size_t i = 0; i < StateSize; ++i)
                trace += M.at(i, i);
            charPoly[k] = -trace / static_cast<T>(k);

            prev = M;
            if (k < StateSize)
            {
                for (std::size_t i = 0; i < StateSize; ++i)
                    prev.at(i, i) += charPoly[k];
                M = clA * prev;
            }
        }

        solvers::DurandKerner<T, StateSize> dk{};
        auto roots = dk.Solve(std::span<const T>{ charPoly.data(), StateSize + 1 });

        for (const auto& root : roots)
            if (std::abs(root) >= T{ 1 })
                return false;

        return true;
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

        auto clA = plant.A - plant.B2 * K;
        return IsSchurStable(clA);
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
