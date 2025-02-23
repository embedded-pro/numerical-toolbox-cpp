#include "numerical/neural_network/activation/ActivationFunction.hpp"
#include "numerical/neural_network/layer/Dense.hpp"
#include "gmock/gmock.h"

namespace
{
    template<typename T>
    class ActivationMock
        : public neural_network::ActivationFunction<T>
    {
    public:
        MOCK_METHOD(T, Forward, (T x), (const, override));
        MOCK_METHOD(T, Backward, (T x), (const, override));
    };

    template<typename T>
    class TestDense
        : public ::testing::Test
    {
    public:
        static constexpr std::size_t InputSize = 3;
        static constexpr std::size_t OutputSize = 2;

        neural_network::Dense<T, InputSize, OutputSize, ActivationMock<T>> dense;
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestDense, TestedTypes);
}

TYPED_TEST(TestDense, placeholder)
{
}
