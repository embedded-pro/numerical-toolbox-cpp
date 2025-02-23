#include "numerical/neural_network/activation/ReLU.hpp"
#include "gmock/gmock.h"

namespace
{
    template<typename T>
    class TestReLU
        : public ::testing::Test
    {
    public:
        neural_network::ReLU<T> activation;
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestReLU, TestedTypes);
}

TYPED_TEST(TestReLU, placeholder)
{
}
