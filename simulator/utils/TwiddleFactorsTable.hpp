#pragma once

#include "numerical/analysis/FastFourierTransform.hpp"
#include <array>
#include <cmath>
#include <numbers>

namespace simulator::utils
{
    template<typename QNumberType, std::size_t Length>
    class TwiddleFactorsTable
        : public ::analysis::TwiddleFactors<QNumberType, Length>
    {
    public:
        TwiddleFactorsTable()
        {
            for (std::size_t k = 0; k < Length; ++k)
            {
                float angle = -std::numbers::pi_v<float> * static_cast<float>(k) / static_cast<float>(Length);
                factors[k] = math::Complex<QNumberType>(std::cos(angle), std::sin(angle));
            }
        }

        math::Complex<QNumberType>& operator[](std::size_t n) override
        {
            return factors[n];
        }

    private:
        std::array<math::Complex<QNumberType>, Length> factors{};
    };
}
