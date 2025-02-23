#include "numerical/neural_network/activation/Tanh.hpp"
#include "gmock/gmock.h"

namespace
{
    template<typename T>
    class TestTanh
        : public ::testing::Test
    {
    public:
        neural_network::Tanh<T> activation;
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestTanh, TestedTypes);
}

TYPED_TEST(TestTanh, placeholder)
{
}
