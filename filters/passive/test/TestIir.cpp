#include "filters/passive/Iir.hpp"
#include "gtest/gtest.h"

namespace
{
    template<typename T>
    class TestIir
        : public ::testing::Test
    {
    public:
        std::optional<filters::passive::Iir<T, 2, 3>> controller;
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestIir, TestedTypes);
}

TYPED_TEST(TestIir, bla)
{
}
