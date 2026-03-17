#pragma once

#include "numerical/analysis/FastFourierTransform.hpp"
#include <cmath>
#include <vector>

namespace simulator::analysis
{
    template<typename QNumberType, std::size_t Length>
    class TwiddleFactorsTable
        : public ::analysis::TwiddleFactors<QNumberType, Length>
    {
    public:
        TwiddleFactorsTable()
        {
            factors.reserve(Length);

            for (std::size_t k = 0; k < Length; ++k)
            {
                float angle = static_cast<float>(-M_PI) * static_cast<float>(k) / static_cast<float>(Length);
                factors.emplace_back(std::cos(angle), std::sin(angle));
            }
        }

        math::Complex<QNumberType>& operator[](std::size_t n) override
        {
            return factors[n];
        }

    private:
        std::vector<math::Complex<QNumberType>> factors;
    };
}
