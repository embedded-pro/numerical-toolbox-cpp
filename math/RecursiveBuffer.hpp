#ifndef MATH_RECURSIVE_BUFFER_H
#define MATH_RECURSIVE_BUFFER_H

#include "math/QNumber.hpp"
#include <array>

namespace math
{
    struct IndexRelative
    {
        explicit IndexRelative(int32_t offset = 0)
            : offset(offset)
        {}

        int32_t offset;
    };

    struct Index
    {
        IndexRelative operator-(int32_t offset) const
        {
            return IndexRelative(offset);
        }

        IndexRelative operator+() const
        {
            return IndexRelative(0);
        }
    };

    namespace detail
    {
        template<typename T>
        inline constexpr bool is_supported_type_v =
            std::disjunction_v<std::is_same<T, float>, is_qnumber<T>>;
    }

    template<typename QNumberType, std::size_t Size>
    class RecursiveBuffer
    {
        static_assert(detail::is_supported_type_v<QNumberType>,
            "RecursiveBuffer only supports float or QNumber types");

    public:
        RecursiveBuffer();

        void Update(QNumberType value);
        QNumberType operator[](const IndexRelative& n) const;

    private:
        std::array<QNumberType, Size> buffer;
    };

    // Implementation

    template<typename QNumberType, std::size_t Size>
    RecursiveBuffer<QNumberType, Size>::RecursiveBuffer()
    {
        buffer.fill(0);
    }

    template<typename QNumberType, std::size_t Size>
    void RecursiveBuffer<QNumberType, Size>::Update(QNumberType value)
    {
        for (auto i = buffer.size() - 1; i > 0; --i)
            buffer[i] = buffer[i - 1];

        buffer[0] = value;
    }

    template<typename QNumberType, std::size_t Size>
    QNumberType RecursiveBuffer<QNumberType, Size>::operator[](const IndexRelative& n) const
    {
        return buffer.at(n.offset);
    }
}

#endif
