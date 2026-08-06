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
