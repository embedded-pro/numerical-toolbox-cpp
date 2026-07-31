#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/LinearTimeInvariant.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/solvers/GaussianElimination.hpp"
#include <cmath>
#include <cstddef>
#include <type_traits>

namespace robust_control
{
    template<typename T, std::size_t StateSize, std::size_t InputSize>
    class SlidingModeControl
    {
        static_assert(std::is_floating_point_v<T>, "SlidingModeControl supports floating-point types");
        static_assert(StateSize > 0 && InputSize > 0, "SlidingModeControl requires positive dimensions");

    public:
        using StateVector = math::Vector<T, StateSize>;
        using InputVector = math::Vector<T, InputSize>;
        using SurfaceMatrix = math::Matrix<T, InputSize, StateSize>;
        using SBMatrix = math::SquareMatrix<T, InputSize>;
        using PlantType = math::LinearTimeInvariant<T, StateSize, InputSize, StateSize>;

        SlidingModeControl(const PlantType& plant,
            const SurfaceMatrix& surface,
            const InputVector& switchGain,
            T phi);

        OPTIMIZE_FOR_SPEED InputVector ComputeControl(const StateVector& x);
        OPTIMIZE_FOR_SPEED InputVector ComputeControl(const StateVector& x, const StateVector& reference);

        [[nodiscard]] InputVector Surface(const StateVector& x) const;
        void SetBoundaryLayer(T phi);

        [[nodiscard]] static T Sat(T s, T phi);

    private:
        OPTIMIZE_FOR_SPEED InputVector ScaledSat(const InputVector& s) const;

        PlantType plant;
        SurfaceMatrix surface;
        SBMatrix sbInv;
        InputVector switchGain;
        T boundaryLayer;
    };

    template<typename T, std::size_t StateSize, std::size_t InputSize>
    SlidingModeControl<T, StateSize, InputSize>::SlidingModeControl(
        const PlantType& plant,
        const SurfaceMatrix& surface,
        const InputVector& switchGain,
        T phi)
        : plant{ plant }
        , surface{ surface }
        , switchGain{ switchGain }
        , boundaryLayer{ phi }
    {
        really_assert(phi > T{ 0 });

        const auto sb = surface * plant.B;
        const auto identity = SBMatrix::Identity();
        sbInv = solvers::SolveSystem<T, InputSize, InputSize>(sb, identity);
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize>
    OPTIMIZE_FOR_SPEED typename SlidingModeControl<T, StateSize, InputSize>::InputVector
    SlidingModeControl<T, StateSize, InputSize>::ComputeControl(const StateVector& x)
    {
        const auto s = surface * x;
        const auto sax = surface * (plant.A * x);
        const auto uEq = sbInv * sax * T{ -1 };
        const auto uSw = sbInv * ScaledSat(s);
        return uEq - uSw;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize>
    OPTIMIZE_FOR_SPEED typename SlidingModeControl<T, StateSize, InputSize>::InputVector
    SlidingModeControl<T, StateSize, InputSize>::ComputeControl(const StateVector& x, const StateVector& reference)
    {
        const auto error = x - reference;
        return ComputeControl(error);
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize>
    typename SlidingModeControl<T, StateSize, InputSize>::InputVector
    SlidingModeControl<T, StateSize, InputSize>::Surface(const StateVector& x) const
    {
        return surface * x;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize>
    void SlidingModeControl<T, StateSize, InputSize>::SetBoundaryLayer(T phi)
    {
        really_assert(phi > T{ 0 });
        boundaryLayer = phi;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize>
    T SlidingModeControl<T, StateSize, InputSize>::Sat(T s, T phi)
    {
        const T ratio = s / phi;
        if (ratio >= T{ 1 })
            return T{ 1 };
        if (ratio <= T{ -1 })
            return T{ -1 };
        return ratio;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize>
    OPTIMIZE_FOR_SPEED typename SlidingModeControl<T, StateSize, InputSize>::InputVector
    SlidingModeControl<T, StateSize, InputSize>::ScaledSat(const InputVector& s) const
    {
        InputVector result{};
        for (std::size_t i = 0; i < InputSize; ++i)
            result.at(i, 0) = switchGain.at(i, 0) * Sat(s.at(i, 0), boundaryLayer);
        return result;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class SlidingModeControl<float, 2, 1>;
#endif
}
