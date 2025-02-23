#include "numerical/neural_network/losses/MeanAbsoluteError.hpp"
#include "gmock/gmock.h"

namespace
{
    template<typename T>
    class TestMeanAbsoluteError
        : public ::testing::Test
    {
    public:
        static constexpr std::size_t NumberOfFeature = 2;
        std::optional<neural_network::MeanAbsoluteError<T, NumberOfFeature>> loss;
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestMeanAbsoluteError, TestedTypes);
}

TYPED_TEST(TestMeanAbsoluteError, placeholder)
{
}
