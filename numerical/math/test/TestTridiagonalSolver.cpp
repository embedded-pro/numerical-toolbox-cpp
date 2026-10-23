#include "numerical/math/TridiagonalSolver.hpp"
#include <array>
#include <cstddef>
#include <gtest/gtest.h>

namespace
{
    using Solver = math::TridiagonalSolver<float, 8>;
    using Band = Solver::Band;

    class TestTridiagonalSolver : public ::testing::Test
    {
    protected:
        Solver solver;
        Band sub{ 0.0f, -1.0f, -1.0f, -1.0f };
        Band diag{ 2.0f, 2.0f, 2.0f, 2.0f };
        Band sup{ -1.0f, -1.0f, -1.0f, 0.0f };

        static void ExpectBandNear(const Band& actual, const Band& expected)
        {
            for (std::size_t k = 0; k < expected.size(); ++k)
                EXPECT_NEAR(actual[k], expected[k], 1e-5f) << "k = " << k;
        }
    };
}

TEST_F(TestTridiagonalSolver, solves_second_difference_system)
{
    ASSERT_TRUE(solver.Factor(sub, diag, sup, 4));

    ExpectBandNear(solver.Solve(Band{ 1.0f, 0.0f, 0.0f, 1.0f }), Band{ 1.0f, 1.0f, 1.0f, 1.0f });
}

TEST_F(TestTridiagonalSolver, one_factorization_serves_several_right_hand_sides)
{
    ASSERT_TRUE(solver.Factor(sub, diag, sup, 4));

    ExpectBandNear(solver.Solve(Band{ 1.0f, 0.0f, 0.0f, 1.0f }), Band{ 1.0f, 1.0f, 1.0f, 1.0f });
    ExpectBandNear(solver.Solve(Band{ 1.0f, 2.0f, 3.0f, 4.0f }), Band{ 4.0f, 7.0f, 8.0f, 6.0f });
}

TEST_F(TestTridiagonalSolver, single_row_system_divides_by_the_diagonal)
{
    ASSERT_TRUE(solver.Factor(Band{}, Band{ 4.0f }, Band{}, 1));

    ExpectBandNear(solver.Solve(Band{ 2.0f }), Band{ 0.5f });
}

TEST_F(TestTridiagonalSolver, vanishing_pivot_is_reported)
{
    const Band ones{ 1.0f, 1.0f, 1.0f, 1.0f };
    const Band minusOnes{ -1.0f, -1.0f, -1.0f, -1.0f };

    EXPECT_FALSE(solver.Factor(minusOnes, ones, minusOnes, 4));
    EXPECT_EQ(solver.Size(), 0);
}

TEST_F(TestTridiagonalSolver, size_outside_range_is_rejected)
{
    EXPECT_FALSE(solver.Factor(sub, diag, sup, 0));
    EXPECT_FALSE(solver.Factor(sub, diag, sup, 9));
}
