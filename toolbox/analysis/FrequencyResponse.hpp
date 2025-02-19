#ifndef ANALYSIS_FREQUENCY_RESPONSE_HPP
#define ANALYSIS_FREQUENCY_RESPONSE_HPP

#include "infra/util/BoundedVector.hpp"
#include "infra/util/MemoryRange.hpp"
#include "toolbox/math/QNumber.hpp"
#include <complex>
#include <tuple>

namespace analysis
{
    template<typename QNumberType, std::size_t NumberOfPoints>
    class FrequencyResponse
    {
        static_assert(math::is_qnumber<QNumberType>::value ||
                          std::is_floating_point<QNumberType>::value,
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
    std::tuple<typename FrequencyResponse<QNumberType, NumberOfPoints>::Vector, typename FrequencyResponse<QNumberType, NumberOfPoints>::Vector, typename FrequencyResponse<QNumberType, NumberOfPoints>::Vector> FrequencyResponse<QNumberType, NumberOfPoints>::Calculate()
    {
        const auto fstart = sampleFrequency / response.max_size();
        const auto fend = sampleFrequency / 2.0f;
        const auto multiplier = std::pow(fend / fstart, 1.0f / (response.max_size() - 1));

        for (auto f = fstart; f <= fend; f *= multiplier)
        {
            auto omega = 2.0f * M_PI * f / sampleFrequency;

            std::complex<float> numerator(0.0, 0.0);
            std::complex<float> denominator(0.0, 0.0);

            for (size_t i = 0; i < b.size(); ++i)
                numerator += math::ToFloat(b[i]) * std::polar<float>(1.0f, -omega * i); // z = e^(jω)

            for (size_t i = 0; i < a.size(); ++i)
                denominator += math::ToFloat(a[i]) * std::polar<float>(1.0f, -omega * i); // z = e^(jω)

            if (denominator == 0.0f)
                denominator.real(1.0f);

            auto h = numerator / denominator;

            frequencies.emplace_back(f);
            response.emplace_back(20.0f * std::log10(std::abs(h)));
            phase.emplace_back(std::arg(h) * 180.0f / M_PI);
        }

        return std::make_tuple(frequencies, response, phase);
    }
}

#endif
