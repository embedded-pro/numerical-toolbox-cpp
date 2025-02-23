#include "numerical/neural_network/losses/BinaryCrossEntropy.hpp"
#include "gmock/gmock.h"

namespace
{
    template<typename T>
    class TestBinaryCrossEntropy
        : public ::testing::Test
    {
    public:
        static constexpr std::size_t NumberOfFeature = 2;
        std::optional<neural_network::BinaryCrossEntropy<T, NumberOfFeature>> loss;
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestBinaryCrossEntropy, TestedTypes);
}

TYPED_TEST(TestBinaryCrossEntropy, placeholder)
{
}
