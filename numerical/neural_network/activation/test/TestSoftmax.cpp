#include "numerical/neural_network/activation/Softmax.hpp"
#include "gmock/gmock.h"

namespace
{
    template<typename T>
    class TestSoftmax
        : public ::testing::Test
    {
    public:
        neural_network::Softmax<T> activation;
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestSoftmax, TestedTypes);
}

TYPED_TEST(TestSoftmax, placeholder)
{
}
