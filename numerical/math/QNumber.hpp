#ifndef MATH_Q_NUMBER_H
#define MATH_Q_NUMBER_H

#include "infra/util/ReallyAssert.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include <chrono>
#include <compare>
#include <cstdint>
#include <limits>
#include <numbers>

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

namespace math
{
    inline constexpr double pi = std::numbers::pi;
    inline constexpr double pi_2 = std::numbers::pi / 2.0;
    inline constexpr double pi_4 = std::numbers::pi / 4.0;

    template<typename T>
    OPTIMIZE_FOR_SPEED float ToFloat(T value)
    {
        if constexpr (std::is_same_v<T, float>)
            return value;
        else
            return value.ToFloat();
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED float Min()
    {
        if constexpr (std::is_same_v<T, float>)
            return std::numeric_limits<float>::min();
        else
            return -0.9999f;
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED float Max()
    {
        if constexpr (std::is_same_v<T, float>)
            return std::numeric_limits<float>::max();
        else
            return 0.9999f;
    }

    template<typename T>
    OPTIMIZE_FOR_SPEED float Lowest()
    {
        if constexpr (std::is_same_v<T, float>)
            return std::numeric_limits<float>::lowest();
        else
            return -0.9999f;
    }

    template<typename IntType, int FractionalBits>
    class QNumber
    {
    public:
        QNumber();
        explicit constexpr QNumber(float f);
        explicit QNumber(IntType rawValue);

        float ToFloat() const;
        IntType RawValue() const;

        OPTIMIZE_FOR_SPEED friend QNumber operator+(const QNumber& lhs, const QNumber& rhs)
        {
#ifdef NUMERICAL_TOOLBOX_ENABLE_ASSERTIONS
            really_assert((rhs.value <= 0 || lhs.value <= std::numeric_limits<IntType>::max() - rhs.value) &&
                          (rhs.value >= 0 || lhs.value >= std::numeric_limits<IntType>::min() - rhs.value));
#endif
            return QNumber(static_cast<IntType>(lhs.value + rhs.value));
        }

        OPTIMIZE_FOR_SPEED friend QNumber operator-(const QNumber& lhs, const QNumber& rhs)
        {
#ifdef NUMERICAL_TOOLBOX_ENABLE_ASSERTIONS
            really_assert((rhs.value >= 0 || lhs.value <= std::numeric_limits<IntType>::max() + rhs.value) &&
                          (rhs.value <= 0 || lhs.value >= std::numeric_limits<IntType>::min() + rhs.value));
#endif
            return QNumber(static_cast<IntType>(lhs.value - rhs.value));
        }

        OPTIMIZE_FOR_SPEED friend QNumber operator*(const QNumber& lhs, const QNumber& rhs)
        {
            auto temp = static_cast<int64_t>(lhs.value) * rhs.value;
            temp >>= FractionalBits;
#ifdef NUMERICAL_TOOLBOX_ENABLE_ASSERTIONS
            really_assert(temp <= std::numeric_limits<IntType>::max() &&
                          temp >= std::numeric_limits<IntType>::min());
#endif
            return QNumber(static_cast<IntType>(temp));
        }

        OPTIMIZE_FOR_SPEED friend QNumber operator/(const QNumber& lhs, const QNumber& rhs)
        {
#ifdef NUMERICAL_TOOLBOX_ENABLE_ASSERTIONS
            really_assert(rhs.value != 0);
#endif
            int64_t numerator = static_cast<int64_t>(lhs.value) << FractionalBits;
            int64_t result = numerator / rhs.value;
#ifdef NUMERICAL_TOOLBOX_ENABLE_ASSERTIONS
            really_assert(result <= std::numeric_limits<IntType>::max() &&
                          result >= std::numeric_limits<IntType>::min());
#endif
            return QNumber(static_cast<IntType>(result));
        }

        QNumber& operator+=(const QNumber& other);
        QNumber& operator-=(const QNumber& other);
        QNumber& operator*=(const QNumber& other);
        QNumber& operator/=(const QNumber& other);

        QNumber operator+() const;
        QNumber operator-() const;

        friend auto operator<=>(const QNumber&, const QNumber&) = default;

        static QNumber FromDuration(std::chrono::microseconds duration)
        {
            return QNumber(static_cast<IntType>(duration.count()));
        }

    private:
        IntType value;

        static constexpr IntType round(float f)
        {
            if (f >= 0.0f)
                return static_cast<IntType>(f + 0.5f);
            else
                return static_cast<IntType>(f - 0.5f);
        }

        static constexpr IntType FloatToFixed(float f)
        {
            really_assert(f >= -1.0f && f < 1.0f);
            return static_cast<IntType>(round(f * (1LL << FractionalBits)));
        }
    };

    using Q31 = QNumber<int32_t, 31>;
    using Q15 = QNumber<int16_t, 15>;

    template<typename T>
    struct is_qnumber : std::false_type
    {};

    template<typename IntType, int FractionalBits>
    struct is_qnumber<QNumber<IntType, FractionalBits>> : std::true_type
    {};

    template<typename T>
    inline constexpr bool is_qnumber_v = is_qnumber<T>::value;

    // Implementation

    template<typename IntType, int FractionalBits>
    QNumber<IntType, FractionalBits>::QNumber()
        : value(0)
    {}

    template<typename IntType, int FractionalBits>
    constexpr QNumber<IntType, FractionalBits>::QNumber(float f)
        : value(FloatToFixed(f))
    {}

    template<typename IntType, int FractionalBits>
    QNumber<IntType, FractionalBits>::QNumber(IntType rawValue)
        : value(rawValue)
    {}

    template<typename IntType, int FractionalBits>
    OPTIMIZE_FOR_SPEED float QNumber<IntType, FractionalBits>::ToFloat() const
    {
        return static_cast<float>(value) / (1LL << FractionalBits);
    }

    template<typename IntType, int FractionalBits>
    OPTIMIZE_FOR_SPEED
        QNumber<IntType, FractionalBits>&
        QNumber<IntType, FractionalBits>::operator+=(const QNumber& other)
    {
#ifdef NUMERICAL_TOOLBOX_ENABLE_ASSERTIONS
        really_assert((other.value <= 0 || value <= std::numeric_limits<IntType>::max() - other.value) &&
                      (other.value >= 0 || value >= std::numeric_limits<IntType>::min() - other.value));
#endif
        value += other.value;
        return *this;
    }

    template<typename IntType, int FractionalBits>
    OPTIMIZE_FOR_SPEED
        QNumber<IntType, FractionalBits>&
        QNumber<IntType, FractionalBits>::operator-=(const QNumber& other)
    {
#ifdef NUMERICAL_TOOLBOX_ENABLE_ASSERTIONS
        really_assert((other.value >= 0 || value <= std::numeric_limits<IntType>::max() + other.value) &&
                      (other.value <= 0 || value >= std::numeric_limits<IntType>::min() + other.value));
#endif
        value -= other.value;
        return *this;
    }

    template<typename IntType, int FractionalBits>
    OPTIMIZE_FOR_SPEED
        QNumber<IntType, FractionalBits>&
        QNumber<IntType, FractionalBits>::operator*=(const QNumber& other)
    {
        auto temp = static_cast<int64_t>(value) * other.value;
        temp >>= FractionalBits;
#ifdef NUMERICAL_TOOLBOX_ENABLE_ASSERTIONS
        really_assert(temp <= std::numeric_limits<IntType>::max() &&
                      temp >= std::numeric_limits<IntType>::min());
#endif
        value = static_cast<IntType>(temp);
        return *this;
    }

    template<typename IntType, int FractionalBits>
    OPTIMIZE_FOR_SPEED
        QNumber<IntType, FractionalBits>&
        QNumber<IntType, FractionalBits>::operator/=(const QNumber& other)
    {
#ifdef NUMERICAL_TOOLBOX_ENABLE_ASSERTIONS
        really_assert(other.value != 0);
#endif
        int64_t numerator = static_cast<int64_t>(value) << FractionalBits;
        int64_t result = numerator / other.value;
#ifdef NUMERICAL_TOOLBOX_ENABLE_ASSERTIONS
        really_assert(result <= std::numeric_limits<IntType>::max() &&
                      result >= std::numeric_limits<IntType>::min());
#endif
        value = static_cast<IntType>(result);
        return *this;
    }

    template<typename IntType, int FractionalBits>
    OPTIMIZE_FOR_SPEED
        QNumber<IntType, FractionalBits>
        QNumber<IntType, FractionalBits>::operator+() const
    {
        return QNumber(static_cast<IntType>(value));
    }

    template<typename IntType, int FractionalBits>
    OPTIMIZE_FOR_SPEED
        QNumber<IntType, FractionalBits>
        QNumber<IntType, FractionalBits>::operator-() const
    {
        return QNumber(static_cast<IntType>(-value));
    }

    template<typename IntType, int FractionalBits>
    OPTIMIZE_FOR_SPEED
        IntType
        QNumber<IntType, FractionalBits>::RawValue() const
    {
        return value;
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class QNumber<int32_t, 31>;
    extern template class QNumber<int16_t, 15>;
#endif
}

#endif
