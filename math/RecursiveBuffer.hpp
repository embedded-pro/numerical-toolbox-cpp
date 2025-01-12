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

    template<typename QNumberType, std::size_t Length>
    class RecursiveBuffer
    {
        static_assert(detail::is_supported_type_v<QNumberType>,
            "RecursiveBuffer only supports float or QNumber types");

    public:
        RecursiveBuffer();

        void Update(QNumberType value);
        void Reset();
        std::size_t Size();
        QNumberType operator[](const IndexRelative& n) const;

    private:
        std::array<QNumberType, Length> buffer;
    };

    // Implementation

    template<typename QNumberType, std::size_t Length>
    RecursiveBuffer<QNumberType, Length>::RecursiveBuffer()
    {
        buffer.fill(0);
    }

    template<typename QNumberType, std::size_t Length>
    void RecursiveBuffer<QNumberType, Length>::Update(QNumberType value)
    {
        for (auto i = buffer.size() - 1; i > 0; --i)
            buffer[i] = buffer[i - 1];

        buffer[0] = value;
    }

    template<typename QNumberType, std::size_t Length>
    void RecursiveBuffer<QNumberType, Length>::Reset()
    {
        buffer.fill(0);
    }

    template<typename QNumberType, std::size_t Length>
    std::size_t RecursiveBuffer<QNumberType, Length>::Size()
    {
        return buffer.size();
    }

    template<typename QNumberType, std::size_t Length>
    QNumberType RecursiveBuffer<QNumberType, Length>::operator[](const IndexRelative& n) const
    {
        return buffer.at(n.offset);
    }
}

#endif
