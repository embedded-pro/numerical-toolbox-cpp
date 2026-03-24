#pragma once
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include <algorithm>
#include <array>

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

namespace math
{
    struct IndexRelative
    {
        explicit constexpr IndexRelative(int32_t offset = 0) noexcept
            : offset(offset)
        {}

        int32_t offset;
    };

    struct Index
    {
        [[nodiscard]] friend constexpr IndexRelative operator-(Index, int32_t offset) noexcept
        {
            return IndexRelative(offset);
        }

        [[nodiscard]] friend constexpr IndexRelative operator+(Index) noexcept
        {
            return IndexRelative(0);
        }
    };

    template<typename QNumberType, std::size_t Length>
    class RecursiveBuffer
    {
        static_assert(detail::is_supported_type_v<QNumberType>,
            "RecursiveBuffer only supports float or QNumber types");

    public:
        constexpr RecursiveBuffer() noexcept;
        RecursiveBuffer& operator=(std::initializer_list<QNumberType> init);

        void Update(QNumberType value) noexcept;
        void Reset() noexcept;
        [[nodiscard]] constexpr std::size_t Size() const noexcept;
        [[nodiscard]] QNumberType operator[](const IndexRelative& n) const noexcept;

    private:
        std::array<QNumberType, Length> buffer{};
    };

    // Implementation

    template<typename QNumberType, std::size_t Length>
    constexpr RecursiveBuffer<QNumberType, Length>::RecursiveBuffer() noexcept = default;

    template<typename QNumberType, std::size_t Length>
    RecursiveBuffer<QNumberType, Length>& RecursiveBuffer<QNumberType, Length>::operator=(std::initializer_list<QNumberType> init)
    {
        really_assert(init.size() <= Length);
        std::ranges::copy(init, buffer.begin());

        if (init.size() < Length)
            std::fill(buffer.begin() + init.size(), buffer.end(), QNumberType(0.0f));

        return *this;
    }

    template<typename QNumberType, std::size_t Length>
    OPTIMIZE_FOR_SPEED void RecursiveBuffer<QNumberType, Length>::Update(QNumberType value) noexcept
    {
        for (std::size_t i = Length - 1; i > 0; --i)
            buffer[i] = buffer[i - 1];

        buffer[0] = value;
    }

    template<typename QNumberType, std::size_t Length>
    void RecursiveBuffer<QNumberType, Length>::Reset() noexcept
    {
        buffer.fill(QNumberType{});
    }

    template<typename QNumberType, std::size_t Length>
    constexpr std::size_t RecursiveBuffer<QNumberType, Length>::Size() const noexcept
    {
        return Length;
    }

    template<typename QNumberType, std::size_t Length>
    OPTIMIZE_FOR_SPEED QNumberType RecursiveBuffer<QNumberType, Length>::operator[](const IndexRelative& n) const noexcept
    {
        return buffer[n.offset];
    }
}
