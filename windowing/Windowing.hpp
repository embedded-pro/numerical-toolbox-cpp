#ifndef WINDOWING_HPP
#define WINDOWING_HPP

#include "math/QNumber.hpp"

namespace windowing
{
    template<typename QNumberType>
    class Window
    {
        static_assert(math::is_qnumber<QNumberType>::value ||
                          std::is_floating_point<QNumberType>::value,
            "Window can only be instantiated with math::QNumber types.");

    public:
        virtual QNumberType operator()(std::size_t n, std::size_t order) = 0;
        virtual QNumberType Power(std::size_t order) = 0;
    };

    template<typename QNumberType>
    class HammingWindow
        : public Window<QNumberType>
    {
    public:
        QNumberType operator()(std::size_t n, std::size_t order) override
        {
            return (0.54f - 0.46f * std::cos(2.0f * M_PI * n / order)) * 0.9999f;
        }

        QNumberType Power(std::size_t order) override
        {
            return QNumberType(0.397f * order);
        }
    };

    template<typename QNumberType>
    class HanningWindow
        : public Window<QNumberType>
    {
    public:
        QNumberType operator()(std::size_t n, std::size_t order) override
        {
            return (0.5f * (1.0f - std::cos(2.0f * M_PI * n / order))) * 0.9999f;
        }

        QNumberType Power(std::size_t order) override
        {
            return QNumberType(0.375f * order);
        }
    };

    template<typename QNumberType>
    class BlackmanWindow
        : public Window<QNumberType>
    {
    public:
        QNumberType operator()(std::size_t n, std::size_t order) override
        {
            return (0.42f - 0.5f * std::cos(2.0f * M_PI * n / order) + 0.08f * std::cos(4.0f * M_PI * n / order)) * 0.9999f;
        }

        QNumberType Power(std::size_t order) override
        {
            return QNumberType(0.305f * order);
        }
    };

    template<typename QNumberType>
    class RectangularWindow
        : public Window<QNumberType>
    {
    public:
        QNumberType operator()(std::size_t, std::size_t) override
        {
            return QNumberType(0.9999f);
        }

        QNumberType Power(std::size_t order) override
        {
            return QNumberType(0.9999f);
        }
    };
}

#endif
