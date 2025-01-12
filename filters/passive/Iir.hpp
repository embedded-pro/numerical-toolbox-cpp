#ifndef FILTERS_PASSIVE_IIR_HPP
#define FILTERS_PASSIVE_IIR_HPP

#include "infra/util/MemoryRange.hpp"
#include "math/QNumber.hpp"
#include "math/RecursiveBuffer.hpp"

namespace filters::passive
{
    template<typename QNumberType, std::size_t P, std::size_t Q>
    class Iir
    {
        static_assert(math::is_qnumber<QNumberType>::value ||
                          std::is_floating_point<QNumberType>::value,
            "Iir can only be instantiated with math::QNumber types.");

    public:
        Iir(infra::MemoryRange<QNumberType> b, infra::MemoryRange<QNumberType> a);

        QNumberType Filter(QNumberType input);
        void Enable();
        void Disable();
        void Reset();

    private:
        bool enabled = true;

        math::Index n;
        infra::MemoryRange<QNumberType> a;
        infra::MemoryRange<QNumberType> b;
        math::RecursiveBuffer<QNumberType, Q> y;
        math::RecursiveBuffer<QNumberType, P> x;
    };

    ////    Implementation    ////

    template<typename QNumberType, std::size_t P, std::size_t Q>
    Iir<QNumberType, P, Q>::Iir(infra::MemoryRange<QNumberType> b, infra::MemoryRange<QNumberType> a)
        : b(b)
        , a(a)
    {
        really_assert(b.size() == x.Size() && a.size() == y.Size());
        Reset();
    }

    template<typename QNumberType, std::size_t P, std::size_t Q>
    QNumberType Iir<QNumberType, P, Q>::Filter(QNumberType input)
    {
        QNumberType feedforward{ 0.0f };
        QNumberType feedback{ 0.0f };

        if (!enabled)
            return input;

        x.Update(input);

        for (auto i = 0; i < b.size(); i++)
            feedforward += b[n - i] * x[n - i];

        for (auto i = 0; i < a.size(); i++)
            feedback += a[n - i] * y[n - i];

        auto output = feedback + feedforward;

        y.Update(output);

        return output;
    }

    template<typename QNumberType, std::size_t P, std::size_t Q>
    void Iir<QNumberType, P, Q>::Enable()
    {
        enabled = true;
    }

    template<typename QNumberType, std::size_t P, std::size_t Q>
    void Iir<QNumberType, P, Q>::Disable()
    {
        enabled = false;
    }

    template<typename QNumberType, std::size_t P, std::size_t Q>
    void Iir<QNumberType, P, Q>::Reset()
    {
        x.Reset();
        y.Reset();
    }
}

#endif
