#pragma once

#include "infra/util/BoundedVector.hpp"
#include "infra/util/MemoryRange.hpp"
#include "numerical/solvers/DurandKerner.hpp"
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <numbers>
#include <type_traits>

namespace analysis
{
    template<typename T, std::size_t MaxOrder, std::size_t MaxGainSteps>
    class RootLocus
    {
        static_assert(std::is_floating_point_v<T>,
            "RootLocus only supports floating-point types");

    public:
        using RootVector = typename infra::BoundedVector<std::complex<T>>::template WithMaxSize<MaxOrder>;
        using GainVector = typename infra::BoundedVector<T>::template WithMaxSize<MaxGainSteps>;
        using LociBranch = typename infra::BoundedVector<std::complex<T>>::template WithMaxSize<MaxGainSteps>;

        struct Result
        {
            std::array<LociBranch, MaxOrder> loci;
            std::size_t activeBranches = 0;
            GainVector gains;
            RootVector openLoopPoles;
            RootVector openLoopZeros;
            RootVector closedLoopPoles;
            T currentGain = T(0);
        };

        Result Calculate(
            infra::MemoryRange<const T> numerator,
            infra::MemoryRange<const T> denominator,
            T currentGain,
            T gainMin = T(0.001),
            T gainMax = T(50)) const;

    private:
        RootVector FindRoots(infra::MemoryRange<const T> polynomial) const;

        static void PolyAdd(
            infra::MemoryRange<const T> base,
            infra::MemoryRange<const T> addend,
            T scale,
            infra::MemoryRange<T> result);
    };

    ////    Implementation    ////

    template<typename T, std::size_t MaxOrder, std::size_t MaxGainSteps>
    typename RootLocus<T, MaxOrder, MaxGainSteps>::RootVector
    RootLocus<T, MaxOrder, MaxGainSteps>::FindRoots(infra::MemoryRange<const T> polynomial) const
    {
        solvers::DurandKerner<T, MaxOrder> solver;
        return solver.Solve(polynomial);
    }

    template<typename T, std::size_t MaxOrder, std::size_t MaxGainSteps>
    void RootLocus<T, MaxOrder, MaxGainSteps>::PolyAdd(
        infra::MemoryRange<const T> base,
        infra::MemoryRange<const T> addend,
        T scale,
        infra::MemoryRange<T> result)
    {
        for (std::size_t i = 0; i < base.size(); ++i)
            result[i] = base[i];

        std::size_t offset = base.size() - addend.size();
        for (std::size_t i = 0; i < addend.size(); ++i)
            result[i + offset] += scale * addend[i];
    }

    template<typename T, std::size_t MaxOrder, std::size_t MaxGainSteps>
    typename RootLocus<T, MaxOrder, MaxGainSteps>::Result
    RootLocus<T, MaxOrder, MaxGainSteps>::Calculate(
        infra::MemoryRange<const T> numerator,
        infra::MemoryRange<const T> denominator,
        T currentGain,
        T gainMin,
        T gainMax) const
    {
        Result result;
        result.currentGain = currentGain;

        result.openLoopPoles = FindRoots(denominator);
        if (numerator.size() > 1)
            result.openLoopZeros = FindRoots(numerator);

        std::size_t polyOrder = denominator.size() - 1;
        result.activeBranches = polyOrder;

        T logMin = std::log10(gainMin);
        T logMax = std::log10(gainMax);

        std::array<T, MaxOrder + 1> charPolyStorage{};

        for (std::size_t step = 0; step < MaxGainSteps; ++step)
        {
            T logK = logMin + (logMax - logMin) * static_cast<T>(step) / static_cast<T>(MaxGainSteps - 1);
            T K = std::pow(T(10), logK);
            result.gains.push_back(K);

            auto charPoly = infra::MakeRange(charPolyStorage.data(), charPolyStorage.data() + denominator.size());
            PolyAdd(denominator, numerator, K, charPoly);

            auto roots = FindRoots(infra::MemoryRange<const T>(charPoly));
            for (std::size_t r = 0; r < polyOrder && r < roots.size(); ++r)
                result.loci[r].push_back(roots[r]);
        }

        std::fill(charPolyStorage.begin(), charPolyStorage.end(), T(0));
        auto closedPoly = infra::MakeRange(charPolyStorage.data(), charPolyStorage.data() + denominator.size());
        PolyAdd(denominator, numerator, currentGain, closedPoly);
        result.closedLoopPoles = FindRoots(infra::MemoryRange<const T>(closedPoly));

        return result;
    }

    extern template class RootLocus<float, 5, 100>;
}
