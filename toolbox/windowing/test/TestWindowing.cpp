#include "toolbox/math/QNumber.hpp"
#include "toolbox/windowing/Windowing.hpp"
#include <gtest/gtest.h>

namespace
{
    template<typename QNumType>
    class WindowingTest
        : public ::testing::Test
    {
    protected:
        static constexpr float kEpsilon = 1e-3f;
        std::unique_ptr<windowing::Window<QNumType>> window;
    };

    using WindowingTypes = ::testing::Types<math::Q31, math::Q15, float>;
    TYPED_TEST_SUITE(WindowingTest, WindowingTypes);
}

TYPED_TEST(WindowingTest, HammingWindowValues)
{
    this->window = std::make_unique<windowing::HammingWindow<TypeParam>>();

    struct TestCase
    {
        std::size_t n;
        std::size_t order;
        float expected;
    } testCases[] = {
        { 0, 8, 0.08f },
        { 4, 8, 0.999f },
        { 8, 8, 0.08f },
        { 2, 8, 0.54f }
    };

    for (const auto& testCase : testCases)
    {
        TypeParam result = (*this->window)(testCase.n, testCase.order);
        EXPECT_NEAR(math::ToFloat(result), testCase.expected, this->kEpsilon)
            << "Failed for n=" << testCase.n
            << ", order=" << testCase.order;
    }
}

TYPED_TEST(WindowingTest, HanningWindowValues)
{
    this->window = std::make_unique<windowing::HanningWindow<TypeParam>>();

    struct TestCase
    {
        std::size_t n;
        std::size_t order;
        float expected;
    } testCases[] = {
        { 0, 8, 0.0f },
        { 4, 8, 0.999f },
        { 8, 8, 0.0f },
        { 2, 8, 0.5f }
    };

    for (const auto& testCase : testCases)
    {
        TypeParam result = (*this->window)(testCase.n, testCase.order);
        EXPECT_NEAR(math::ToFloat(result), testCase.expected, this->kEpsilon)
            << "Failed for n=" << testCase.n
            << ", order=" << testCase.order;
    }
}

TYPED_TEST(WindowingTest, BlackmanWindowValues)
{
    this->window = std::make_unique<windowing::BlackmanWindow<TypeParam>>();

    struct TestCase
    {
        std::size_t n;
        std::size_t order;
        float expected;
    } testCases[] = {
        { 0, 8, 0.0f },
        { 4, 8, 0.999f },
        { 8, 8, 0.0f }
    };

    for (const auto& testCase : testCases)
    {
        TypeParam result = (*this->window)(testCase.n, testCase.order);
        EXPECT_NEAR(math::ToFloat(result), testCase.expected, this->kEpsilon)
            << "Failed for n=" << testCase.n
            << ", order=" << testCase.order;
    }
}

TYPED_TEST(WindowingTest, RectangularWindowValues)
{
    this->window = std::make_unique<windowing::RectangularWindow<TypeParam>>();

    for (std::size_t n = 0; n <= 8; n += 2)
    {
        TypeParam result = (*this->window)(n, 8);
        EXPECT_NEAR(math::ToFloat(result), 0.999f, this->kEpsilon)
            << "Failed for n=" << n;
    }
}

TYPED_TEST(WindowingTest, WindowSymmetry)
{
    std::vector<std::unique_ptr<windowing::Window<TypeParam>>> windows;
    windows.push_back(std::make_unique<windowing::HammingWindow<TypeParam>>());
    windows.push_back(std::make_unique<windowing::HanningWindow<TypeParam>>());
    windows.push_back(std::make_unique<windowing::BlackmanWindow<TypeParam>>());

    const std::size_t order = 16;

    for (const auto& window : windows)
    {
        for (std::size_t n = 0; n < order / 2; ++n)
        {
            TypeParam left = (*window)(n, order);
            TypeParam right = (*window)(order - n, order);
            EXPECT_NEAR(math::ToFloat(left), math::ToFloat(right), this->kEpsilon)
                << "Symmetry failed at n=" << n;
        }
    }
}
