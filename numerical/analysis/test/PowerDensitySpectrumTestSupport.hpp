#pragma once

#include "numerical/analysis/FastFourierTransform.hpp"
#include "numerical/windowing/Windowing.hpp"

namespace analysis::test
{
    template<typename QNumberType>
    class WindowStub
        : public windowing::Window<QNumberType>
    {
    public:
        QNumberType operator()(std::size_t n, std::size_t order) override
        {
            return QNumberType(0.5f);
        }

        QNumberType Power(std::size_t order) override
        {
            return QNumberType(0.25f);
        }
    };

    template<typename QNumberType, std::size_t Length>
    class TwiddleFactorsStub
        : public analysis::TwiddleFactors<QNumberType, Length>
    {
    public:
        math::Complex<QNumberType>& operator[](std::size_t n) override
        {
            static math::Complex<QNumberType> factor(QNumberType(1.0f), QNumberType(0.0f));
            return factor;
        }
    };

    template<typename QNumberType, std::size_t Length>
    class FftStub
        : public analysis::FastFourierTransform<QNumberType>
    {
    public:
        using VectorComplex = typename analysis::FastFourierTransform<QNumberType>::VectorComplex;
        using VectorReal = typename analysis::FastFourierTransform<QNumberType>::VectorReal;

        explicit FftStub(analysis::TwiddleFactors<QNumberType, Length / 2>&)
        {}

        VectorComplex& Forward(VectorReal& input) override
        {
            result.clear();

            for (std::size_t i = 0; i < Length; ++i)
            {
                if (i == 0)
                    result.push_back(math::Complex<QNumberType>(QNumberType(0.5f), QNumberType(0.0f)));
                else
                    result.push_back(math::Complex<QNumberType>(QNumberType(0.0f), QNumberType(0.0f)));
            }
            return result;
        }

        VectorReal& Inverse(VectorComplex& input) override
        {
            timeResult.clear();
            for (std::size_t i = 0; i < Length; ++i)
                timeResult.push_back(QNumberType(0.0f));
            return timeResult;
        }

    private:
        typename VectorComplex::template WithMaxSize<Length> result;
        typename VectorReal::template WithMaxSize<Length> timeResult;
    };
}
