#include "numerical/neural_network/activation/Sigmoid.hpp"
#include "gmock/gmock.h"

namespace
{
    template<typename T>
    class TestSigmoid
        : public ::testing::Test
    {
    public:
        neural_network::Sigmoid<T> activation;
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestSigmoid, TestedTypes);
}

TYPED_TEST(TestSigmoid, placeholder)
{
}
