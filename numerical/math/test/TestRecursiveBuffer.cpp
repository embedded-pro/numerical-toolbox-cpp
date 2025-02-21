#include "numerical/math/QNumber.hpp"
#include "numerical/math/RecursiveBuffer.hpp"
#include <array>
#include <gtest/gtest.h>

namespace
{
    template<typename T>
    class RecursiveBufferTest
        : public ::testing::Test
    {
    public:
        static constexpr std::size_t BufferSize = 4;
        math::RecursiveBuffer<T, BufferSize> buffer;
        const float tolerance = 0.001f;
    };

    using BufferTypes = ::testing::Types<float, math::Q31, math::Q15>;
    TYPED_TEST_SUITE(RecursiveBufferTest, BufferTypes);
}

TYPED_TEST(RecursiveBufferTest, DefaultConstructor)
{
    math::Index n;
    for (std::size_t i = 0; i < this->BufferSize; ++i)
        EXPECT_NEAR(math::ToFloat(this->buffer[n - i]), 0.0f, this->tolerance)
            << "Buffer not zero-initialized at index " << i;
}

TYPED_TEST(RecursiveBufferTest, SingleUpdate)
{
    math::Index n;
    TypeParam value(0.5f);
    this->buffer.Update(value);

    EXPECT_NEAR(math::ToFloat(this->buffer[n - 0]), 0.5f, this->tolerance);
    for (std::size_t i = 1; i < this->BufferSize; ++i)
        EXPECT_NEAR(math::ToFloat(this->buffer[n - i]), 0.0f, this->tolerance)
            << "Non-zero value found at index " << i;
}

TYPED_TEST(RecursiveBufferTest, MultipleUpdates)
{
    math::Index n;
    std::array<float, 4> values = { 0.1f, 0.2f, 0.3f, 0.4f };

    for (float val : values)
        this->buffer.Update(TypeParam(val));

    for (size_t i = 0; i < values.size(); ++i)
        EXPECT_NEAR(
            math::ToFloat(this->buffer[n - i]),
            values[values.size() - 1 - i], this->tolerance)
            << "Incorrect value at index " << i;
}

TYPED_TEST(RecursiveBufferTest, ShiftingBehavior)
{
    math::Index n;

    for (std::size_t i = 1; i <= this->BufferSize; ++i)
        this->buffer.Update(TypeParam(static_cast<float>(i) * 0.1f));

    TypeParam newValue(0.5f);
    this->buffer.Update(newValue);

    EXPECT_NEAR(math::ToFloat(this->buffer[n - 0]), 0.5f, this->tolerance);
    EXPECT_NEAR(math::ToFloat(this->buffer[n - 1]), 0.4f, this->tolerance);
    EXPECT_NEAR(math::ToFloat(this->buffer[n - 2]), 0.3f, this->tolerance);
    EXPECT_NEAR(math::ToFloat(this->buffer[n - 3]), 0.2f, this->tolerance);
}
