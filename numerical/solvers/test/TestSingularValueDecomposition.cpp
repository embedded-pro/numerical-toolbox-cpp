#include "numerical/math/Tolerance.hpp"
#include "numerical/solvers/SingularValueDecomposition.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    class TestSingularValueDecomposition : public ::testing::Test
    {
    protected:
        solvers::SingularValueDecomposition<float, 4, 3> svd{};
        solvers::SingularValueDecomposition<float, 3, 3> svdSquare{};

        math::Matrix<float, 4, 3> a43{
            { 1.0f, 2.0f, 3.0f },
            { 4.0f, 5.0f, 6.0f },
            { 7.0f, 8.0f, 10.0f },
            { 1.0f, 0.0f, 2.0f }
        };
    };
}

TEST_F(TestSingularValueDecomposition, reconstructs_A)
{
    svd.Decompose(a43);

    auto u = svd.U();
    auto sig = svd.SingularValues();
    auto v = svd.V();

    math::Matrix<float, 3, 3> sigMat{};
    for (std::size_t i = 0; i < 3; ++i)
        sigMat.at(i, i) = sig.at(i, 0);

    auto reconstructed = u * sigMat * v.Transpose();

    for (std::size_t i = 0; i < 4; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            EXPECT_NEAR(reconstructed.at(i, j), a43.at(i, j), 1e-4f);
}

TEST_F(TestSingularValueDecomposition, singular_values_descending_and_nonnegative)
{
    svd.Decompose(a43);
    auto sig = svd.SingularValues();

    EXPECT_GE(sig.at(0, 0), sig.at(1, 0));
    EXPECT_GE(sig.at(1, 0), sig.at(2, 0));
    EXPECT_GE(sig.at(2, 0), 0.0f);
}

TEST_F(TestSingularValueDecomposition, U_and_V_orthonormal)
{
    svd.Decompose(a43);

    auto u = svd.U();
    auto v = svd.V();

    auto utu = u.Transpose() * u;
    auto vtv = v.Transpose() * v;

    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
        {
            float expected = (i == j) ? 1.0f : 0.0f;
            EXPECT_NEAR(utu.at(i, j), expected, 1e-4f);
            EXPECT_NEAR(vtv.at(i, j), expected, 1e-4f);
        }
}

TEST_F(TestSingularValueDecomposition, diagonal_matrix_gives_absolute_diagonal)
{
    math::Matrix<float, 3, 3> d{
        { 3.0f, 0.0f, 0.0f },
        { 0.0f, -1.0f, 0.0f },
        { 0.0f, 0.0f, 2.0f }
    };

    svdSquare.Decompose(d);
    auto sig = svdSquare.SingularValues();

    EXPECT_NEAR(sig.at(0, 0), 3.0f, 1e-4f);
    EXPECT_NEAR(sig.at(1, 0), 2.0f, 1e-4f);
    EXPECT_NEAR(sig.at(2, 0), 1.0f, 1e-4f);
}

TEST_F(TestSingularValueDecomposition, sigma_squared_matches_eig_AtA)
{
    svd.Decompose(a43);
    auto sig = svd.SingularValues();

    auto ata = a43.Transpose() * a43;
    float traceAtA = ata.at(0, 0) + ata.at(1, 1) + ata.at(2, 2);

    float sumSigSq = 0.0f;
    for (std::size_t i = 0; i < 3; ++i)
        sumSigSq += sig.at(i, 0) * sig.at(i, 0);

    EXPECT_NEAR(sumSigSq, traceAtA, 1e-2f);
}

TEST_F(TestSingularValueDecomposition, pseudo_inverse_satisfies_moore_penrose)
{
    svd.Decompose(a43);
    auto aPlus = svd.PseudoInverse(1e-6f);

    auto aaPlusA = a43 * aPlus * a43;

    for (std::size_t i = 0; i < 4; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            EXPECT_NEAR(aaPlusA.at(i, j), a43.at(i, j), 1e-4f);
}

TEST_F(TestSingularValueDecomposition, pseudo_inverse_solves_least_squares)
{
    math::Matrix<float, 4, 3> aOvd{
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f },
        { 1.0f, 1.0f, 1.0f }
    };

    math::Vector<float, 4> b{ { 1.0f }, { 2.0f }, { 3.0f }, { 6.5f } };

    svd.Decompose(aOvd);
    auto xSvd = svd.SolveLeastSquares(b);

    EXPECT_NEAR(xSvd.at(0, 0), 1.125f, 1e-3f);
    EXPECT_NEAR(xSvd.at(1, 0), 2.125f, 1e-3f);
    EXPECT_NEAR(xSvd.at(2, 0), 3.125f, 1e-3f);
}

TEST_F(TestSingularValueDecomposition, rank_detection_thresholds_small_sigma)
{
    math::Matrix<float, 3, 3> rankTwo{
        { 1.0f, 2.0f, 3.0f },
        { 4.0f, 5.0f, 6.0f },
        { 7.0f, 8.0f, 9.0f }
    };

    svdSquare.Decompose(rankTwo);
    std::size_t r = svdSquare.Rank(0.1f);

    EXPECT_EQ(r, 2u);
}

TEST_F(TestSingularValueDecomposition, condition_number_matches_known)
{
    math::Matrix<float, 3, 3> d{
        { 3.0f, 0.0f, 0.0f },
        { 0.0f, 2.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f }
    };

    svdSquare.Decompose(d);
    float cond = svdSquare.ConditionNumber();

    EXPECT_NEAR(cond, 3.0f, 1e-4f);
}

TEST_F(TestSingularValueDecomposition, known_2x2_svd)
{
    math::Matrix<float, 3, 3> m{
        { 4.0f, 3.0f, 0.0f },
        { 0.0f, 5.0f, 0.0f },
        { 0.0f, 0.0f, 0.0f }
    };

    svdSquare.Decompose(m);
    auto sig = svdSquare.SingularValues();

    EXPECT_NEAR(sig.at(0, 0), std::sqrt(40.0f), 1e-3f);
    EXPECT_NEAR(sig.at(1, 0), std::sqrt(10.0f), 1e-3f);
    EXPECT_NEAR(sig.at(2, 0), 0.0f, 1e-3f);
}

TEST_F(TestSingularValueDecomposition, ZeroDiagonalBidiagonalBlockDiagonalizes)
{
    math::Matrix<float, 3, 3> single{};
    single.at(0, 1) = 1.0f;

    ASSERT_TRUE(svdSquare.Decompose(single));

    EXPECT_NEAR(svdSquare.SingularValues().at(0, 0), 1.0f, math::Tolerance<float>());
    EXPECT_NEAR(svdSquare.SingularValues().at(1, 0), 0.0f, math::Tolerance<float>());
    EXPECT_NEAR(svdSquare.SingularValues().at(2, 0), 0.0f, math::Tolerance<float>());
}

TEST_F(TestSingularValueDecomposition, ZeroDiagonalBlockReconstructsInput)
{
    math::Matrix<float, 3, 3> single{};
    single.at(0, 1) = 1.0f;

    ASSERT_TRUE(svdSquare.Decompose(single));

    const auto& u = svdSquare.U();
    const auto& v = svdSquare.V();

    for (std::size_t r = 0; r < 3; ++r)
        for (std::size_t c = 0; c < 3; ++c)
        {
            float acc = 0.0f;
            for (std::size_t k = 0; k < 3; ++k)
                acc += u.at(r, k) * svdSquare.SingularValues().at(k, 0) * v.at(c, k);

            EXPECT_NEAR(acc, single.at(r, c), 1e-5f);
        }
}

TEST_F(TestSingularValueDecomposition, TwoNonzeroSuperdiagonalsDiagonalize)
{
    math::Matrix<float, 3, 3> bidiagonal{};
    bidiagonal.at(0, 1) = 1.0f;
    bidiagonal.at(1, 2) = 2.0f;

    ASSERT_TRUE(svdSquare.Decompose(bidiagonal));

    EXPECT_NEAR(svdSquare.SingularValues().at(0, 0), 2.0f, 1e-5f);
    EXPECT_NEAR(svdSquare.SingularValues().at(1, 0), 1.0f, 1e-5f);
    EXPECT_NEAR(svdSquare.SingularValues().at(2, 0), 0.0f, 1e-5f);
}

TEST_F(TestSingularValueDecomposition, rank_and_condition_number_of_tall_matrix)
{
    ASSERT_TRUE(svd.Decompose(a43));

    EXPECT_EQ(svd.Rank(1e-5f), 3u);
    EXPECT_GT(svd.ConditionNumber(), 1.0f);
}

TEST_F(TestSingularValueDecomposition, rank_drops_for_rank_deficient_tall_matrix)
{
    math::Matrix<float, 4, 3> deficient{
        { 1.0f, 2.0f, 3.0f },
        { 2.0f, 4.0f, 6.0f },
        { 3.0f, 6.0f, 9.0f },
        { 4.0f, 8.0f, 12.0f }
    };

    ASSERT_TRUE(svd.Decompose(deficient));
    EXPECT_EQ(svd.Rank(1e-4f), 1u);
}

TEST_F(TestSingularValueDecomposition, pseudo_inverse_of_square_matrix_solves_system)
{
    math::Matrix<float, 3, 3> invertible{
        { 2.0f, 0.0f, 0.0f },
        { 0.0f, 4.0f, 0.0f },
        { 0.0f, 0.0f, 5.0f }
    };

    ASSERT_TRUE(svdSquare.Decompose(invertible));

    const auto pinv = svdSquare.PseudoInverse(1e-6f);

    EXPECT_NEAR(pinv.at(0, 0), 0.5f, 1e-4f);
    EXPECT_NEAR(pinv.at(1, 1), 0.25f, 1e-4f);
    EXPECT_NEAR(pinv.at(2, 2), 0.2f, 1e-4f);
}

TEST_F(TestSingularValueDecomposition, trailing_zero_diagonal_block_is_deflated_left)
{
    math::Matrix<float, 4, 3> trailingZero{};
    trailingZero.at(0, 0) = 2.0f;
    trailingZero.at(1, 2) = 3.0f;

    ASSERT_TRUE(svd.Decompose(trailingZero));

    EXPECT_NEAR(svd.SingularValues().at(0, 0), 3.0f, 1e-4f);
    EXPECT_NEAR(svd.SingularValues().at(1, 0), 2.0f, 1e-4f);
    EXPECT_NEAR(svd.SingularValues().at(2, 0), 0.0f, 1e-4f);
}
