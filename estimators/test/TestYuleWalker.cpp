#include "estimators/YuleWalker.hpp"
#include <gmock/gmock.h>
#include <utility>

namespace
{
    template<typename T, std::size_t N>
    class MockSolver
        : public solvers::Solver<T, N>
    {
    public:
        using SolutionVectorType = typename solvers::Solver<T, N>::SolutionVector;
        using InputMatrixType = typename solvers::Solver<T, N>::InputMatrix;
        using InputVectorType = typename solvers::Solver<T, N>::InputVector;

        MOCK_METHOD((SolutionVectorType), Solve, (const InputMatrixType&, const InputVectorType&), (override));
    };

    template<typename T>
    class YuleWalkerTest
        : public ::testing::Test
    {
    protected:
        static constexpr size_t Samples = 4;
        static constexpr size_t Order = 2;
        using MockSolverType = MockSolver<T, Order>;
        using EstimatorType = estimators::YuleWalker<T, Samples, Order>;
        using TimeSeriesVector = math::Matrix<T, Samples, 1>;
        using DesignMatrix = math::Matrix<T, Samples, Order>;
        using VectorType = math::Vector<T, Order>;

        void SetUp() override
        {
            solver = std::make_unique<MockSolverType>();
            estimator = std::make_unique<EstimatorType>(*solver);
        }

        static T MakeValue(float f)
        {
            return T(f);
        }

        VectorType MakeVector(float a, float b)
        {
            return VectorType{
                { MakeValue(a) },
                { MakeValue(b) }
            };
        }

        TimeSeriesVector MakeTimeSeries(float a, float b, float c, float d)
        {
            return TimeSeriesVector{
                { MakeValue(a) },
                { MakeValue(b) },
                { MakeValue(c) },
                { MakeValue(d) }
            };
        }

        DesignMatrix MakeDesignMatrix(std::pair<float, float> a, std::pair<float, float> b,
            std::pair<float, float> c, std::pair<float, float> d)
        {
            return DesignMatrix{
                { MakeValue(a.first), MakeValue(a.second) },
                { MakeValue(b.first), MakeValue(b.second) },
                { MakeValue(c.first), MakeValue(c.second) },
                { MakeValue(d.first), MakeValue(d.second) }
            };
        }

        std::unique_ptr<MockSolverType> solver;
        std::unique_ptr<EstimatorType> estimator;
    };

    using TestTypes = ::testing::Types<float>;
    TYPED_TEST_SUITE(YuleWalkerTest, TestTypes);
}

TYPED_TEST(YuleWalkerTest, FitComputesCorrectInputs)
{
    auto y = this->MakeTimeSeries(1.0f, 0.4f, 0.15f, 0.1f);
    auto X = this->MakeDesignMatrix(
        std::make_pair(0.0f, 0.0f),
        std::make_pair(1.0f, 0.0f),
        std::make_pair(0.4f, 1.0f),
        std::make_pair(0.15f, 0.4f));

    EXPECT_CALL(*this->solver, Solve(::testing::_, ::testing::_))
        .WillOnce(::testing::Return(this->MakeVector(0.5f, -0.25f)));

    this->estimator->Fit(X, y);
}

TYPED_TEST(YuleWalkerTest, PredictUsesCoefficientsCorrectly)
{
    auto y = this->MakeTimeSeries(0.0f, 0.0f, 0.0f, 0.0f);
    auto X = this->MakeDesignMatrix(
        std::make_pair(0.0f, 0.0f),
        std::make_pair(0.0f, 0.0f),
        std::make_pair(0.0f, 0.0f),
        std::make_pair(0.0f, 0.0f));

    EXPECT_CALL(*this->solver, Solve(::testing::_, ::testing::_))
        .WillOnce(::testing::Return(this->MakeVector(1.0f, -1.0f)));

    this->estimator->Fit(X, y);

    auto prediction = this->estimator->Predict(this->MakeVector(0.5f, 0.25f));
    EXPECT_NEAR(math::ToFloat(prediction), 0.25f, 0.01f);
}

TYPED_TEST(YuleWalkerTest, ConstantTimeSeriesHandling)
{
    auto y = this->MakeTimeSeries(1.0f, 1.0f, 1.0f, 1.0f);
    auto X = this->MakeDesignMatrix(
        std::make_pair(0.0f, 0.0f),
        std::make_pair(1.0f, 0.0f),
        std::make_pair(1.0f, 1.0f),
        std::make_pair(1.0f, 1.0f));

    EXPECT_CALL(*this->solver, Solve(::testing::_, ::testing::_))
        .WillOnce(::testing::Return(this->MakeVector(0.0f, 0.0f)));

    this->estimator->Fit(X, y);
    auto prediction = this->estimator->Predict(this->MakeVector(1.0f, 1.0f));
    EXPECT_NEAR(math::ToFloat(prediction), 1.0f, 0.01f);
}

TYPED_TEST(YuleWalkerTest, CenteredDataComputation)
{
    auto y = this->MakeTimeSeries(2.0f, 1.4f, 1.15f, 1.1f);
    auto X = this->MakeDesignMatrix(
        std::make_pair(0.0f, 0.0f),
        std::make_pair(1.0f, 0.0f),
        std::make_pair(0.4f, 1.0f),
        std::make_pair(0.15f, 0.4f));

    EXPECT_CALL(*this->solver, Solve(::testing::_, ::testing::_))
        .WillOnce(::testing::Return(this->MakeVector(0.5f, -0.25f)));

    this->estimator->Fit(X, y);
}
