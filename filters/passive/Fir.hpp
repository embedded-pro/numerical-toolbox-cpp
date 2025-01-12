#ifndef FILTERS_PASSIVE_FIR_HPP
#define FILTERS_PASSIVE_FIR_HPP

#include "math/QNumber.hpp"
#include "math/RecursiveBuffer.hpp"

namespace filters::passive
{
    template<typename QNumberType, std::size_t N>
    class Fir
    {
        static_assert(math::is_qnumber<QNumberType>::value ||
                          std::is_floating_point<QNumberType>::value,
            "Fir can only be instantiated with math::QNumber types.");

    public:
        Fir(math::RecursiveBuffer<QNumberType, N> b);

        QNumberType Filter(QNumberType input);
        void Enable();
        void Disable();
        void Reset();

    private:
        bool enabled = true;

        math::Index n;
        math::RecursiveBuffer<QNumberType, N> b;
        math::RecursiveBuffer<QNumberType, N> x;
    };

    ////    Implementation    ////

    template<typename QNumberType, std::size_t N>
    Fir<QNumberType, N>::Fir(math::RecursiveBuffer<QNumberType, N> b)
        : b(b)
    {
        Reset();
    }

    template<typename QNumberType, std::size_t N>
    QNumberType Fir<QNumberType, N>::Filter(QNumberType input)
    {
        QNumberType output{ 0.0f };

        if (!enabled)
            return input;

        x.Update(input);

        for (auto i = 0; i < b.Size(); i++)
            output += b[n - i] * x[n - i];

        return output;
    }

    template<typename QNumberType, std::size_t N>
    void Fir<QNumberType, N>::Enable()
    {
        enabled = true;
    }

    template<typename QNumberType, std::size_t N>
    void Fir<QNumberType, N>::Disable()
    {
        enabled = false;
    }

    template<typename QNumberType, std::size_t N>
    void Fir<QNumberType, N>::Reset()
    {
        x.Reset();
    }
}

#endif
