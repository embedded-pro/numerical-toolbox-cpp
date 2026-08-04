#include "numerical/math/QNumber.hpp"
#include "numerical/math/RecursiveBuffer.hpp"
#include "numerical/math/Tolerance.hpp"
#include <array>
#include <gtest/gtest.h>

namespace
{
    template<typename T>
    class RecursiveBufferTest
        : public ::testing::Test
    {
    protected:
        static constexpr std::size_t BufferSize = 4;
        math::RecursiveBuffer<T, BufferSize> buffer;

        void SetUp() override
        {
            buffer.Reset();
        }
    };

    using BufferTypes = ::testing::Types<float, math::Q31, math::Q15>;
    TYPED_TEST_SUITE(RecursiveBufferTest, BufferTypes);
}

TYPED_TEST(RecursiveBufferTest, DefaultInitializationAllSlotsZero)
{
    math::Index n;

    for (std::size_t i = 0; i < this->BufferSize; ++i)
        EXPECT_NEAR(math::ToFloat(this->buffer[n - i]), 0.0f, math::Tolerance<float>());
}

TYPED_TEST(RecursiveBufferTest, SizeReturnsTemplateLength)
{
    EXPECT_EQ(this->buffer.Size(), this->BufferSize);
}

TYPED_TEST(RecursiveBufferTest, SingleUpdatePlacesValueAtCurrentIndex)
{
    math::Index n;
    this->buffer.Update(TypeParam(0.5f));

    EXPECT_NEAR(math::ToFloat(this->buffer[+n]), 0.5f, math::Tolerance<float>());
}

TYPED_TEST(RecursiveBufferTest, SingleUpdateLeavesOlderSlotsZero)
{
    math::Index n;
    this->buffer.Update(TypeParam(0.5f));

    for (std::size_t i = 1; i < this->BufferSize; ++i)
        EXPECT_NEAR(math::ToFloat(this->buffer[n - i]), 0.0f, math::Tolerance<float>());
}

TYPED_TEST(RecursiveBufferTest, MultipleUpdatesShiftValuesCorrectly)
{
    math::Index n;
    std::array<float, 4> values = { 0.1f, 0.2f, 0.3f, 0.4f };

    for (float val : values)
        this->buffer.Update(TypeParam(val));

    for (std::size_t i = 0; i < values.size(); ++i)
        EXPECT_NEAR(
            math::ToFloat(this->buffer[n - i]),
            values[values.size() - 1 - i],
            math::Tolerance<float>());
}

TYPED_TEST(RecursiveBufferTest, OverflowDropsOldestValue)
{
    math::Index n;

    for (std::size_t i = 1; i <= this->BufferSize; ++i)
        this->buffer.Update(TypeParam(static_cast<float>(i) * 0.1f));

    this->buffer.Update(TypeParam(0.5f));

    EXPECT_NEAR(math::ToFloat(this->buffer[n - 0]), 0.5f, math::Tolerance<float>());
    EXPECT_NEAR(math::ToFloat(this->buffer[n - 1]), 0.4f, math::Tolerance<float>());
    EXPECT_NEAR(math::ToFloat(this->buffer[n - 2]), 0.3f, math::Tolerance<float>());
    EXPECT_NEAR(math::ToFloat(this->buffer[n - 3]), 0.2f, math::Tolerance<float>());
}

TYPED_TEST(RecursiveBufferTest, ResetAfterUpdateClearsAllSlots)
{
    math::Index n;
    this->buffer.Update(TypeParam(0.5f));
    this->buffer.Update(TypeParam(0.25f));
    this->buffer.Reset();

    for (std::size_t i = 0; i < this->BufferSize; ++i)
        EXPECT_NEAR(math::ToFloat(this->buffer[n - i]), 0.0f, math::Tolerance<float>());
}

TYPED_TEST(RecursiveBufferTest, ResetRestoredStateMatchesFreshInstance)
{
    math::Index n;
    math::RecursiveBuffer<TypeParam, 4> fresh;

    this->buffer.Update(TypeParam(0.75f));
    this->buffer.Reset();

    for (std::size_t i = 0; i < this->BufferSize; ++i)
        EXPECT_NEAR(
            math::ToFloat(this->buffer[n - i]),
            math::ToFloat(fresh[n - i]),
            math::Tolerance<float>());
}

TYPED_TEST(RecursiveBufferTest, AssignmentFromInitializerListSetsSlots)
{
    math::Index n;
    this->buffer = { TypeParam(0.1f), TypeParam(0.2f), TypeParam(0.3f), TypeParam(0.4f) };

    EXPECT_NEAR(math::ToFloat(this->buffer[n - 0]), 0.1f, math::Tolerance<float>());
    EXPECT_NEAR(math::ToFloat(this->buffer[n - 1]), 0.2f, math::Tolerance<float>());
    EXPECT_NEAR(math::ToFloat(this->buffer[n - 2]), 0.3f, math::Tolerance<float>());
    EXPECT_NEAR(math::ToFloat(this->buffer[n - 3]), 0.4f, math::Tolerance<float>());
}

TYPED_TEST(RecursiveBufferTest, PartialAssignmentZeroFillsRemainingSlots)
{
    math::Index n;
    this->buffer = { TypeParam(0.5f), TypeParam(0.25f) };

    EXPECT_NEAR(math::ToFloat(this->buffer[n - 0]), 0.5f, math::Tolerance<float>());
    EXPECT_NEAR(math::ToFloat(this->buffer[n - 1]), 0.25f, math::Tolerance<float>());
    EXPECT_NEAR(math::ToFloat(this->buffer[n - 2]), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(math::ToFloat(this->buffer[n - 3]), 0.0f, math::Tolerance<float>());
}

TYPED_TEST(RecursiveBufferTest, DeterministicOutputForIdenticalInputSequence)
{
    math::Index n;
    math::RecursiveBuffer<TypeParam, 4> second;

    std::array<float, 4> seq = { 0.1f, 0.3f, 0.7f, 0.9f };

    for (float v : seq)
    {
        this->buffer.Update(TypeParam(v));
        second.Update(TypeParam(v));
    }

    for (std::size_t i = 0; i < this->BufferSize; ++i)
        EXPECT_FLOAT_EQ(
            math::ToFloat(this->buffer[n - i]),
            math::ToFloat(second[n - i]));
}

TYPED_TEST(RecursiveBufferTest, TwoIndependentInstancesDoNotInterfere)
{
    math::Index n;
    math::RecursiveBuffer<TypeParam, 4> other;

    this->buffer.Update(TypeParam(0.9f));
    other.Update(TypeParam(0.1f));

    EXPECT_NEAR(math::ToFloat(this->buffer[n - 0]), 0.9f, math::Tolerance<float>());
    EXPECT_NEAR(math::ToFloat(other[n - 0]), 0.1f, math::Tolerance<float>());
}
