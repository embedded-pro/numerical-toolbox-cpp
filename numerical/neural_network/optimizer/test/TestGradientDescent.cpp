#include "numerical/math/QNumber.hpp"
#include "numerical/neural_network/optimizer/GradientDescent.hpp"
#include "gmock/gmock.h"

namespace
{
    template<typename T>
    class VectorMatcher
    {
    public:
        using Vector = math::Vector<T, 2>;

        explicit VectorMatcher(const Vector& expected)
            : expected(expected)
        {}

        bool operator()(const Vector& actual) const
        {
            for (size_t i = 0; i < 2; ++i)
                if (math::ToFloat(actual[i]) != math::ToFloat(expected[i]))
                    return false;
            return true;
        }

    private:
        Vector expected;
    };

    template<typename T>
    testing::Matcher<const math::Vector<T, 2>&> VectorEq(const math::Vector<T, 2>& expected)
    {
        return testing::Truly(VectorMatcher<T>(expected));
    }

    template<typename T, std::size_t Features>
    class MockLoss
        : public neural_network::Loss<T, Features>
    {
    public:
        using Vector = typename neural_network::Loss<T, Features>::Vector;

        MOCK_METHOD(T, Cost, (const Vector& parameters), (override));
        MOCK_METHOD(Vector, Gradient, (const Vector& parameters), (override));
    };

    template<typename T>
    class TestGradientDescent
        : public ::testing::Test
    {
    protected:
        static constexpr std::size_t Features = 2;
        using Vector = math::Vector<T, Features>;

        void SetUp() override
        {
            params.learningRate = T(0.1f);
            params.maxIterations = 100;
        }

        typename neural_network::GradientDescent<T, Features>::Parameters params;
        MockLoss<T, Features> loss;

        Vector MakeVector(float x, float y)
        {
            Vector result;
            result[0] = T(std::clamp(x, -0.99f, 0.99f));
            result[1] = T(std::clamp(y, -0.99f, 0.99f));
            return result;
        }

        T MakeScalar(float value)
        {
            return T(std::clamp(value, -0.99f, 0.99f));
        }
    };

    using TestedTypes = ::testing::Types<float, math::Q15, math::Q31>;
    TYPED_TEST_SUITE(TestGradientDescent, TestedTypes);
}

TYPED_TEST(TestGradientDescent, performs_expected_number_of_iterations)
{
    using Vector = typename TestGradientDescent<TypeParam>::Vector;

    auto initialGuess = this->MakeVector(0.0f, 0.0f);
    auto gradientResult = this->MakeVector(0.01f, 0.01f);
    auto costResult = this->MakeScalar(0.05f);

    EXPECT_CALL(this->loss, Cost(::testing::_))
        .Times(this->params.maxIterations + 1)
        .WillRepeatedly(::testing::Return(costResult));

    EXPECT_CALL(this->loss, Gradient(::testing::_))
        .Times(this->params.maxIterations)
        .WillRepeatedly(::testing::Return(gradientResult));

    neural_network::GradientDescent<TypeParam, TestGradientDescent<TypeParam>::Features>
        optimizer(this->params);

    auto result = optimizer.Minimize(initialGuess, this->loss);

    EXPECT_EQ(result.iterations, this->params.maxIterations);
}

TYPED_TEST(TestGradientDescent, calls_methods_in_correct_order)
{
    using Vector = typename TestGradientDescent<TypeParam>::Vector;

    auto initialGuess = this->MakeVector(0.0f, 0.0f);
    auto gradientResult = this->MakeVector(0.1f, 0.1f);
    auto costResult = this->MakeScalar(0.5f);

    {
        ::testing::InSequence seq;

        EXPECT_CALL(this->loss, Cost(VectorEq(initialGuess)))
            .WillOnce(::testing::Return(costResult));

        EXPECT_CALL(this->loss, Gradient(::testing::_))
            .WillOnce(::testing::Return(gradientResult));

        EXPECT_CALL(this->loss, Cost(::testing::_))
            .WillOnce(::testing::Return(costResult));

        EXPECT_CALL(this->loss, Gradient(::testing::_))
            .WillOnce(::testing::Return(gradientResult));

        EXPECT_CALL(this->loss, Cost(::testing::_))
            .WillOnce(::testing::Return(costResult));
    }

    this->params.maxIterations = 2;
    neural_network::GradientDescent<TypeParam, TestGradientDescent<TypeParam>::Features>
        optimizer(this->params);

    optimizer.Minimize(initialGuess, this->loss);
}
