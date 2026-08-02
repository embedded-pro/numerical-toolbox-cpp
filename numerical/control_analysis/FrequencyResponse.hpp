#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "infra/util/BoundedVector.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include <cmath>
#include <complex>
#include <limits>
#include <numbers>
#include <span>
#include <tuple>

namespace control_analysis
{
    template<typename T, std::size_t NumberOfPoints>
    class FrequencyResponse
    {
        static_assert(std::is_floating_point_v<T>, "FrequencyResponse supports floating-point types");

    public:
        using Vector = typename infra::BoundedVector<T>::template WithMaxSize<NumberOfPoints>;

        FrequencyResponse(std::span<T> b, std::span<T> a, T sampleFrequency);
        std::tuple<Vector, Vector, Vector> Calculate();

    private:
        Vector frequencies;
        Vector response;
        Vector phase;
        std::span<T> a;
        std::span<T> b;
        T sampleFrequency;
    };

    ////    Implementation    ////

    template<typename T, std::size_t NumberOfPoints>
    FrequencyResponse<T, NumberOfPoints>::FrequencyResponse(std::span<T> b, std::span<T> a, T sampleFrequency)
        : b(b)
        , a(a)
        , sampleFrequency(sampleFrequency)
    {
    }

    template<typename T, std::size_t NumberOfPoints>
    OPTIMIZE_FOR_SPEED
        std::tuple<typename FrequencyResponse<T, NumberOfPoints>::Vector, typename FrequencyResponse<T, NumberOfPoints>::Vector, typename FrequencyResponse<T, NumberOfPoints>::Vector>
        FrequencyResponse<T, NumberOfPoints>::Calculate()
    {
        const auto maxSize = static_cast<T>(response.max_size());
        const auto fstart = sampleFrequency / maxSize;
        const auto fend = sampleFrequency / static_cast<T>(2);
        const auto multiplier = static_cast<T>(std::pow(static_cast<double>(fend / fstart), 1.0 / (static_cast<double>(maxSize) - 1.0)));

        for (std::size_t n = 0; n < response.max_size(); ++n)
        {
            auto f = fstart * static_cast<T>(std::pow(static_cast<double>(multiplier), static_cast<double>(n)));
            auto omega = static_cast<T>(2) * static_cast<T>(std::numbers::pi) * f / sampleFrequency;

            std::complex<T> numerator(static_cast<T>(0), static_cast<T>(0));
            std::complex<T> denominator(static_cast<T>(0), static_cast<T>(0));

            for (std::size_t i = 0; i < b.size(); ++i)
                numerator += b[i] * std::polar(static_cast<T>(1), -omega * static_cast<T>(i));

            for (std::size_t i = 0; i < a.size(); ++i)
                denominator += a[i] * std::polar(static_cast<T>(1), -omega * static_cast<T>(i));

            if (denominator == static_cast<T>(0))
                denominator.real(static_cast<T>(1));

            auto h = numerator / denominator;
            auto magnitude = std::max(std::abs(h), std::numeric_limits<T>::min());

            frequencies.emplace_back(f);
            response.emplace_back(static_cast<T>(20) * std::log10(magnitude));
            phase.emplace_back(std::arg(h) * static_cast<T>(180) / static_cast<T>(std::numbers::pi));
        }

        return std::make_tuple(frequencies, response, phase);
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class FrequencyResponse<float, 64>;
    extern template class FrequencyResponse<float, 128>;
#endif
}
