#include "solvers/LevinsonDurbin.hpp"
#include <gtest/gtest.h>

namespace
{
    template<typename T>
    bool AreVectorsNear(const math::Vector<T, 2>& a,
        const math::Vector<T, 2>& b,
        float epsilon = 1e-2f)
    {
        for (size_t i = 0; i < 2; ++i)
        {
            if (std::abs(math::ToFloat(a.at(i, 0)) - math::ToFloat(b.at(i, 0))) >= epsilon)
                return false;
        }
        return true;
    }

    template<typename T>
    class LevinsonDurbinTest
        : public ::testing::Test
    {
    protected:
        static constexpr size_t N = 2;
        using SolverType = solvers::LevinsonDurbin<T, N>;
        using MatrixType = math::Matrix<T, N, N>;
        using VectorType = math::Vector<T, N>;
        using ToeplitzType = math::ToeplitzMatrix<T, N>;

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

        MatrixType MakeMatrix(float a11, float a12, float a21, float a22)
        {
            return MatrixType{
                { MakeValue(a11), MakeValue(a12) },
                { MakeValue(a21), MakeValue(a22) }
            };
        }
    };

    using TestTypes = ::testing::Types<float /*, math::Q15, math::Q31*/>;
    TYPED_TEST_SUITE(LevinsonDurbinTest, TestTypes);
}

TYPED_TEST(LevinsonDurbinTest, SolveSymmetricToeplitz)
{
    typename TestFixture::SolverType solver;

    auto r = this->MakeVector(0.02f, 0.01f);
    auto toeplitz = math::ToeplitzMatrix<TypeParam, TestFixture::N>(r);
    auto A = toeplitz.ToFullMatrix();
    auto b = this->MakeVector(0.01f, 0.005f);
    auto x = solver.Solve(A, b);

    auto expected = this->MakeVector(0.5f, 0.0f);
    EXPECT_TRUE(AreVectorsNear(x, expected));
}

TYPED_TEST(LevinsonDurbinTest, NonToeplitzError)
{
    typename TestFixture::SolverType solver;

    auto A = this->MakeMatrix(0.01f, 0.02f, 0.03f, 0.04f);
    auto b = this->MakeVector(0.01f, 0.005f);

    EXPECT_DEATH(solver.Solve(A, b), "");
}
