#include "numerical/neural_network/activation/LeakyReLU.hpp"
#include "gmock/gmock.h"

namespace
{
    template<typename T>
    class TestLeakyReLU
        : public ::testing::Test
    {
    public:
        neural_network::LeakyReLU<T> activation;
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestLeakyReLU, TestedTypes);
}

TYPED_TEST(TestLeakyReLU, placeholder)
{
}
