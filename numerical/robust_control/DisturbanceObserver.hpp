// Copyright (c) 2024, Numerical Toolbox Contributors. All rights reserved.
#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/filters/passive/BiquadCascade.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/LinearTimeInvariant.hpp"
#include "numerical/math/Math.hpp"
#include "numerical/math/Matrix.hpp"
#include <array>
#include <cstddef>
#include <type_traits>

namespace robust_control
{
    template<typename T,
        std::size_t StateSize,
        std::size_t InputSize,
        std::size_t OutputSize>
    class DisturbanceObserver
    {
        static_assert(std::is_floating_point_v<T>, "DisturbanceObserver supports floating-point types");
        static_assert(StateSize > 0 && InputSize > 0 && OutputSize > 0,
            "DisturbanceObserver requires positive dimensions");
        static_assert(InputSize == OutputSize,
            "DisturbanceObserver requires InputSize == OutputSize for per-channel disturbance pairing");

    public:
        using PlantType = math::LinearTimeInvariant<T, StateSize, InputSize, OutputSize>;
        using InputVector = math::Vector<T, InputSize>;
        using OutputVector = math::Vector<T, OutputSize>;

        using QCoeffs = filters::passive::BiquadCoeffs<T>;

        DisturbanceObserver(const PlantType& nominalPlant, const QCoeffs& q);

        OPTIMIZE_FOR_SPEED InputVector Compute(const InputVector& nominalControl,
            const OutputVector& measuredOutput);

        [[nodiscard]] const InputVector& Disturbance() const;

        void Reset();

    private:
        static std::array<T, InputSize> ComputeDcGainInverses(const PlantType& plant);
        static std::array<filters::passive::BiquadCascade<T, 1>, InputSize> MakeQFilters(const QCoeffs& q);

        PlantType nominalPlant;
        std::array<T, InputSize> dcGainInv;
        std::array<filters::passive::BiquadCascade<T, 1>, InputSize> qInvFilters;
        std::array<filters::passive::BiquadCascade<T, 1>, InputSize> qFilters;
        InputVector disturbance{};
        InputVector appliedPrev{};
    };

    namespace detail
    {
        template<typename T, std::size_t N>
        T ComputeSisoSteadyStateDcGain(
            const math::LinearTimeInvariant<T, N, 1, 1>& plant)
        {
            math::Vector<T, N> x{};
            math::Vector<T, 1> u{};
            u.at(0, 0) = T{ 1 };
            for (std::size_t k{ 0 }; k < 512; ++k)
                x = plant.Step(x, u);
            const auto y{ plant.Output(x, u) };
            return y.at(0, 0);
        }
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>
    std::array<T, InputSize>
    DisturbanceObserver<T, StateSize, InputSize, OutputSize>::ComputeDcGainInverses(
        const PlantType& plant)
    {
        std::array<T, InputSize> result{};
        for (std::size_t ch{ 0 }; ch < InputSize; ++ch)
        {
            using SisoPlant = math::LinearTimeInvariant<T, StateSize, 1, 1>;
            SisoPlant siso{};
            for (std::size_t r{ 0 }; r < StateSize; ++r)
            {
                for (std::size_t c{ 0 }; c < StateSize; ++c)
                    siso.A.at(r, c) = plant.A.at(r, c);
                siso.B.at(r, 0) = plant.B.at(r, ch);
                siso.C.at(0, r) = plant.C.at(ch, r);
            }
            siso.D.at(0, 0) = plant.D.at(ch, ch);

            const T gain{ detail::ComputeSisoSteadyStateDcGain(siso) };
            result[ch] = (gain == T{ 0 }) ? T{ 1 } : T{ 1 } / gain;
        }
        return result;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>
    DisturbanceObserver<T, StateSize, InputSize, OutputSize>::DisturbanceObserver(
        const PlantType& nominalPlant,
        const QCoeffs& q)
        : nominalPlant{ nominalPlant }
        , dcGainInv{ ComputeDcGainInverses(nominalPlant) }
        , qInvFilters{ MakeQFilters(q) }
        , qFilters{ MakeQFilters(q) }
    {}

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>
    std::array<filters::passive::BiquadCascade<T, 1>, InputSize>
    DisturbanceObserver<T, StateSize, InputSize, OutputSize>::MakeQFilters(const QCoeffs& q)
    {
        return [&]<std::size_t... Is>(std::index_sequence<Is...>)
            -> std::array<filters::passive::BiquadCascade<T, 1>, InputSize>
        {
            return { ((void)Is, filters::passive::BiquadCascade<T, 1>{ { q } })... };
        }(std::make_index_sequence<InputSize>{});
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>
    OPTIMIZE_FOR_SPEED typename DisturbanceObserver<T, StateSize, InputSize, OutputSize>::InputVector
    DisturbanceObserver<T, StateSize, InputSize, OutputSize>::Compute(
        const InputVector& nominalControl,
        const OutputVector& measuredOutput)
    {
        for (std::size_t ch{ 0 }; ch < InputSize; ++ch)
        {
            const T a{ qInvFilters[ch].Filter(measuredOutput.at(ch, 0) * dcGainInv[ch]) };
            const T b{ qFilters[ch].Filter(appliedPrev.at(ch, 0)) };
            disturbance.at(ch, 0) = a - b;
        }

        InputVector u{};
        for (std::size_t ch{ 0 }; ch < InputSize; ++ch)
            u.at(ch, 0) = nominalControl.at(ch, 0) - disturbance.at(ch, 0);

        appliedPrev = u;
        return u;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>
    const typename DisturbanceObserver<T, StateSize, InputSize, OutputSize>::InputVector&
    DisturbanceObserver<T, StateSize, InputSize, OutputSize>::Disturbance() const
    {
        return disturbance;
    }

    template<typename T, std::size_t StateSize, std::size_t InputSize, std::size_t OutputSize>
    void DisturbanceObserver<T, StateSize, InputSize, OutputSize>::Reset()
    {
        disturbance = InputVector{};
        appliedPrev = InputVector{};
        for (std::size_t ch{ 0 }; ch < InputSize; ++ch)
        {
            qInvFilters[ch].Reset();
            qFilters[ch].Reset();
        }
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class DisturbanceObserver<float, 1, 1, 1>;
#endif
}
