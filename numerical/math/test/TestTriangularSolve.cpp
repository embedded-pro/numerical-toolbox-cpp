#include "numerical/math/Tolerance.hpp"
#include "numerical/math/TriangularSolve.hpp"
#include <gtest/gtest.h>

namespace
{
    class TriangularSolveTest : public ::testing::Test
    {
    protected:
        math::Matrix<float, 3, 3> r{
            { 2.0f, -1.0f, 3.0f },
            { 0.0f, 4.0f, 1.0f },
            { 0.0f, 0.0f, 5.0f }
        };
        math::Vector<float, 3> x{ { 1.0f }, { -2.0f }, { 3.0f } };
    };
}

TEST_F(TriangularSolveTest, SolvesUpperTriangularSystem)
{
    math::Vector<float, 3> c;
    for (std::size_t i = 0; i < 3; ++i)
    {
        float sum = 0.0f;
        for (std::size_t j = 0; j < 3; ++j)
            sum += r.at(i, j) * x.at(j, 0);
        c.at(i, 0) = sum;
    }

    auto solution = math::SolveUpperTriangular(r, c);

    for (std::size_t i = 0; i < 3; ++i)
        EXPECT_NEAR(solution.at(i, 0), x.at(i, 0), math::Tolerance<float>());
}

TEST_F(TriangularSolveTest, SolvesUnitLowerTriangularSystem)
{
    math::Matrix<float, 3, 3> l{
        { 1.0f, 0.0f, 0.0f },
        { -2.0f, 1.0f, 0.0f },
        { 3.0f, 4.0f, 1.0f }
    };

    math::Vector<float, 3> c;
    for (std::size_t i = 0; i < 3; ++i)
    {
        float sum = 0.0f;
        for (std::size_t j = 0; j < 3; ++j)
            sum += l.at(i, j) * x.at(j, 0);
        c.at(i, 0) = sum;
    }

    auto solution = math::SolveUnitLowerTriangular(l, c);

    for (std::size_t i = 0; i < 3; ++i)
        EXPECT_NEAR(solution.at(i, 0), x.at(i, 0), math::Tolerance<float>());
}

TEST_F(TriangularSolveTest, IdentityReturnsRightHandSide)
{
    auto identity = math::SquareMatrix<float, 3>::Identity();
    math::Vector<float, 3> b{ { 7.0f }, { -3.0f }, { 2.0f } };

    auto solution = math::SolveUpperTriangular(identity, b);

    for (std::size_t i = 0; i < 3; ++i)
        EXPECT_NEAR(solution.at(i, 0), b.at(i, 0), math::Tolerance<float>());
}
