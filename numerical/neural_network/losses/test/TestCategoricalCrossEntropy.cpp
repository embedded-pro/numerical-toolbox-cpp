#include "numerical/neural_network/losses/CategoricalCrossEntropy.hpp"
#include "gmock/gmock.h"

namespace
{
    template<typename T>
    class TestCategoricalCrossEntropy
        : public ::testing::Test
    {
    public:
        static constexpr std::size_t NumberOfFeature = 2;
        std::optional<neural_network::CategoricalCrossEntropy<T, NumberOfFeature>> loss;
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestCategoricalCrossEntropy, TestedTypes);
}

TYPED_TEST(TestCategoricalCrossEntropy, placeholder)
{
}
