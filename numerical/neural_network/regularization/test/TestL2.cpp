#include "numerical/neural_network/regularization/L2.hpp"
#include "gmock/gmock.h"

namespace
{
    template<typename T>
    class TestL2
        : public ::testing::Test
    {
    public:
        static constexpr std::size_t Size = 4;

        neural_network::L2<T, Size> regularization{ T(0.001f) };
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestL2, TestedTypes);
}

TYPED_TEST(TestL2, placeholder)
{
}
