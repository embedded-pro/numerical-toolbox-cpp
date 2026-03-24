#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/math/CompilerOptimizations.hpp"

#include "infra/util/BoundedVector.hpp"
#include "infra/util/MemoryRange.hpp"
#include "numerical/math/QNumber.hpp"
#include <cmath>
#include <complex>
#include <tuple>

namespace analysis
{
    template<typename QNumberType, std::size_t NumberOfPoints>
    class FrequencyResponse
    {
        static_assert(math::is_qnumber_v<QNumberType> ||
                          std::is_floating_point_v<QNumberType>,
            "FrequencyResponse can only be instantiated with math::QNumber types.");

    public:
        using Vector = infra::BoundedVector<float>::WithMaxSize<NumberOfPoints>;

        FrequencyResponse(infra::MemoryRange<QNumberType> b, infra::MemoryRange<QNumberType> a, float sampleFrequency);
        std::tuple<Vector, Vector, Vector> Calculate();

    private:
        Vector frequencies;
        Vector response;
        Vector phase;
        infra::MemoryRange<QNumberType> a;
        infra::MemoryRange<QNumberType> b;
        float sampleFrequency;
    };

    ////    Implementation    ////

    template<typename QNumberType, std::size_t NumberOfPoints>
    FrequencyResponse<QNumberType, NumberOfPoints>::FrequencyResponse(infra::MemoryRange<QNumberType> b, infra::MemoryRange<QNumberType> a, float sampleFrequency)
        : b(b)
        , a(a)
        , sampleFrequency(sampleFrequency)
    {
    }

    template<typename QNumberType, std::size_t NumberOfPoints>
    OPTIMIZE_FOR_SPEED
        std::tuple<typename FrequencyResponse<QNumberType, NumberOfPoints>::Vector, typename FrequencyResponse<QNumberType, NumberOfPoints>::Vector, typename FrequencyResponse<QNumberType, NumberOfPoints>::Vector>
        FrequencyResponse<QNumberType, NumberOfPoints>::Calculate()
    {
        const auto maxSize = static_cast<double>(response.max_size());
        const auto fstart = static_cast<float>(sampleFrequency / maxSize);
        const auto fend = sampleFrequency / 2.0f;
        const auto multiplier = static_cast<float>(std::pow(fend / fstart, 1.0 / (maxSize - 1.0)));

        for (std::size_t n = 0; n < response.max_size(); ++n)
        {
            auto f = fstart * std::pow(multiplier, static_cast<float>(n));
            auto omega = 2.0f * static_cast<float>(math::pi) * f / sampleFrequency;

            std::complex<float> numerator(0.0f, 0.0f);
            std::complex<float> denominator(0.0f, 0.0f);

            for (std::size_t i = 0; i < b.size(); ++i)
                numerator += math::ToFloat(b[i]) * std::polar(1.0f, -omega * static_cast<float>(i));

            for (std::size_t i = 0; i < a.size(); ++i)
                denominator += math::ToFloat(a[i]) * std::polar(1.0f, -omega * static_cast<float>(i));

            if (denominator == 0.0f)
                denominator.real(1.0f);

            auto h = numerator / denominator;

            frequencies.emplace_back(f);
            response.emplace_back(20.0f * std::log10(std::abs(h)));
            phase.emplace_back(std::arg(h) * 180.0f / static_cast<float>(math::pi));
        }

        return std::make_tuple(frequencies, response, phase);
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class FrequencyResponse<float, 64>;
    extern template class FrequencyResponse<float, 128>;
#endif
}
