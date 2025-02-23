#include "numerical/neural_network/losses/MeanSquaredError.hpp"
#include "gmock/gmock.h"

namespace
{
    template<typename T>
    class TestMeanSquaredError
        : public ::testing::Test
    {
    public:
        static constexpr std::size_t NumberOfFeature = 2;
        std::optional<neural_network::MeanSquaredError<T, NumberOfFeature>> loss;
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestMeanSquaredError, TestedTypes);
}

TYPED_TEST(TestMeanSquaredError, placeholder)
{
}
