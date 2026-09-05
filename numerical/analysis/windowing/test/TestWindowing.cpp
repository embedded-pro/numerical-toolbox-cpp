#include "numerical/analysis/windowing/Windowing.hpp"
#include "numerical/math/QNumber.hpp"
#include <array>
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

TYPED_TEST(WindowingTest, HammingWindowPower)
{
    windowing::HammingWindow<TypeParam> w;
    EXPECT_NEAR(math::ToFloat(w.Power(8)), 0.397f, this->kEpsilon);
}

TYPED_TEST(WindowingTest, HanningWindowPower)
{
    windowing::HanningWindow<TypeParam> w;
    EXPECT_NEAR(math::ToFloat(w.Power(8)), 0.375f, this->kEpsilon);
}

TYPED_TEST(WindowingTest, BlackmanWindowPower)
{
    windowing::BlackmanWindow<TypeParam> w;
    EXPECT_NEAR(math::ToFloat(w.Power(8)), 0.305f, this->kEpsilon);
}

TYPED_TEST(WindowingTest, RectangularWindowPower)
{
    windowing::RectangularWindow<TypeParam> w;
    EXPECT_NEAR(math::ToFloat(w.Power(8)), 0.999f, this->kEpsilon);
}

namespace
{
    class WindowPowerFloatTest
        : public ::testing::Test
    {
    protected:
        windowing::HammingWindow<float> hamming;
        windowing::HanningWindow<float> hanning;
        windowing::BlackmanWindow<float> blackman;
        windowing::RectangularWindow<float> rectangular;

        static float DirectMeanSquarePower(windowing::Window<float>& window, std::size_t order)
        {
            float sum = 0.0f;
            for (std::size_t n = 0; n < order; ++n)
            {
                const float value = window(n, order);
                sum += value * value;
            }
            return sum / static_cast<float>(order);
        }
    };
}

TEST_F(WindowPowerFloatTest, PowerMatchesDirectFiniteSumForEveryLength)
{
    std::array<windowing::Window<float>*, 4> windows{ &hamming, &hanning, &blackman, &rectangular };

    for (auto* window : windows)
        for (std::size_t order : { 2u, 4u, 8u, 64u, 256u })
            EXPECT_NEAR(window->Power(order), DirectMeanSquarePower(*window, order), 1e-6f);
}

TEST_F(WindowPowerFloatTest, ShortHannWindowIsNotTheAsymptoticConstant)
{
    EXPECT_NEAR(hanning.Power(2), 0.4999f, 1e-4f);
    EXPECT_NEAR(hanning.Power(256), 0.3749f, 1e-3f);
}
