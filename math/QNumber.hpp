#ifndef MATH_Q_NUMBER_H
#define MATH_Q_NUMBER_H

#include "infra/util/ReallyAssert.hpp"
#include <cmath>
#include <chrono>
#include <cstdint>
#include <limits>

namespace math
{
    template<typename IntType, int FractionalBits>
    class QNumber
    {
    public:
        QNumber();
        QNumber(float f);
        explicit QNumber(IntType rawValue);

        float ToFloat() const;
        IntType RawValue() const;

        QNumber operator+(const QNumber& other) const;
        QNumber operator-(const QNumber& other) const;
        QNumber operator*(const QNumber& other) const;
        QNumber operator/(const QNumber& other) const;
        QNumber& operator+=(const QNumber& other);
        QNumber& operator-=(const QNumber& other);
        QNumber& operator*=(const QNumber& other);
        QNumber& operator/=(const QNumber& other);

        QNumber operator+() const;
        QNumber operator-() const;

        bool operator==(const QNumber& other) const;
        bool operator<(const QNumber& other) const;
        bool operator<=(const QNumber& other) const;
        bool operator>(const QNumber& other) const;
        bool operator>=(const QNumber& other) const;

        static QNumber FromDuration(std::chrono::microseconds duration)
        {
            return QNumber(static_cast<IntType>(duration.count()));
        }

    private:
        IntType value;

        static IntType FloatToFixed(float f)
        {
            really_assert(f >= -1.0f && f < 1.0f);
            return static_cast<IntType>(std::round(f * (1LL << FractionalBits)));
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

    // Implementation

    template<typename IntType, int FractionalBits>
    QNumber<IntType, FractionalBits>::QNumber()
        : value(0)
    {}

    template<typename IntType, int FractionalBits>
    QNumber<IntType, FractionalBits>::QNumber(float f)
        : value(FloatToFixed(f))
    {}

    template<typename IntType, int FractionalBits>
    QNumber<IntType, FractionalBits>::QNumber(IntType rawValue)
        : value(rawValue)
    {}

    template<typename IntType, int FractionalBits>
    float QNumber<IntType, FractionalBits>::ToFloat() const
    {
        return static_cast<float>(value) / (1LL << FractionalBits);
    }

    template<typename IntType, int FractionalBits>
    QNumber<IntType, FractionalBits>
    QNumber<IntType, FractionalBits>::operator+(const QNumber& other) const
    {
        really_assert((other.value <= 0 || value <= std::numeric_limits<IntType>::max() - other.value) &&
                      (other.value >= 0 || value >= std::numeric_limits<IntType>::min() - other.value));

        return QNumber(static_cast<IntType>(value + other.value));
    }

    template<typename IntType, int FractionalBits>
    QNumber<IntType, FractionalBits>
    QNumber<IntType, FractionalBits>::operator-(const QNumber& other) const
    {
        really_assert((other.value >= 0 || value <= std::numeric_limits<IntType>::max() + other.value) &&
                    (other.value <= 0 || value >= std::numeric_limits<IntType>::min() + other.value));

        return QNumber(static_cast<IntType>(value - other.value));
    }

    template<typename IntType, int FractionalBits>
    QNumber<IntType, FractionalBits>
    QNumber<IntType, FractionalBits>::operator*(const QNumber& other) const
    {
        auto temp = static_cast<int64_t>(value) * other.value;
        temp >>= FractionalBits;

        really_assert(temp <= std::numeric_limits<IntType>::max() &&
                    temp >= std::numeric_limits<IntType>::min());
        return QNumber(static_cast<IntType>(temp));
    }

    template<typename IntType, int FractionalBits>
    QNumber<IntType, FractionalBits>
    QNumber<IntType, FractionalBits>::operator/(const QNumber& other) const
    {
        really_assert(other.value != 0);

        int64_t numerator = static_cast<int64_t>(value) << FractionalBits;
        int64_t result = numerator / other.value;

        really_assert(result <= std::numeric_limits<IntType>::max() &&
                    result >= std::numeric_limits<IntType>::min());
        return QNumber(static_cast<IntType>(result));
    }

    template<typename IntType, int FractionalBits>
    QNumber<IntType, FractionalBits>&
    QNumber<IntType, FractionalBits>::operator+=(const QNumber& other)
    {
        really_assert((other.value <= 0 || value <= std::numeric_limits<IntType>::max() - other.value) &&
                    (other.value >= 0 || value >= std::numeric_limits<IntType>::min() - other.value));

        value += other.value;
        return *this;
    }

    template<typename IntType, int FractionalBits>
    QNumber<IntType, FractionalBits>&
    QNumber<IntType, FractionalBits>::operator-=(const QNumber& other)
    {
        really_assert((other.value >= 0 || value <= std::numeric_limits<IntType>::max() + other.value) &&
                    (other.value <= 0 || value >= std::numeric_limits<IntType>::min() + other.value));
        value -= other.value;
        return *this;
    }

    template<typename IntType, int FractionalBits>
    QNumber<IntType, FractionalBits>&
    QNumber<IntType, FractionalBits>::operator*=(const QNumber& other)
    {
        auto temp = static_cast<int64_t>(value) * other.value;
        temp >>= FractionalBits;

        really_assert(temp <= std::numeric_limits<IntType>::max() &&
                    temp >= std::numeric_limits<IntType>::min());
        value = static_cast<IntType>(temp);
        return *this;
    }

    template<typename IntType, int FractionalBits>
    QNumber<IntType, FractionalBits>&
    QNumber<IntType, FractionalBits>::operator/=(const QNumber& other)
    {
        really_assert(other.value != 0);

        int64_t numerator = static_cast<int64_t>(value) << FractionalBits;
        int64_t result = numerator / other.value;

        really_assert(result <= std::numeric_limits<IntType>::max() &&
                    result >= std::numeric_limits<IntType>::min());
        value = static_cast<IntType>(result);
        return *this;
    }

    template<typename IntType, int FractionalBits>
    QNumber<IntType, FractionalBits>
    QNumber<IntType, FractionalBits>::operator+() const
    {
        return QNumber(static_cast<IntType>(value));
    }

    template<typename IntType, int FractionalBits>
    QNumber<IntType, FractionalBits>
    QNumber<IntType, FractionalBits>::operator-() const
    {
        return QNumber(static_cast<IntType>(-value));
    }

    template<typename IntType, int FractionalBits>
    bool QNumber<IntType, FractionalBits>::operator==(const QNumber& other) const
    {
        return value == other.value;
    }

    template<typename IntType, int FractionalBits>
    bool QNumber<IntType, FractionalBits>::operator<(const QNumber& other) const
    {
        return value < other.value;
    }

    template<typename IntType, int FractionalBits>
    bool QNumber<IntType, FractionalBits>::operator<=(const QNumber& other) const
    {
        return value <= other.value;
    }

    template<typename IntType, int FractionalBits>
    bool QNumber<IntType, FractionalBits>::operator>(const QNumber& other) const
    {
        return value > other.value;
    }

    template<typename IntType, int FractionalBits>
    bool QNumber<IntType, FractionalBits>::operator>=(const QNumber& other) const
    {
        return value >= other.value;
    }

    template<typename IntType, int FractionalBits>
    IntType QNumber<IntType, FractionalBits>::RawValue() const
    {
        return value;
    }
}

#endif
