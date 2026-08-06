#include "numerical/math/Tolerance.hpp"
#include "numerical/solvers/QrDecomposition.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    class TestQrDecomposition : public ::testing::Test
    {
    protected:
        solvers::QrDecomposition<float, 4, 3> qr{};
        solvers::QrDecomposition<float, 3, 3> qrSquare{};
    };
}

TEST_F(TestQrDecomposition, reconstructs_A)
{
    math::Matrix<float, 4, 3> a{
        { 1.0f, 2.0f, 3.0f },
        { 4.0f, 5.0f, 6.0f },
        { 7.0f, 8.0f, 10.0f },
        { 1.0f, 0.0f, 2.0f }
    };

    qr.Decompose(a);
    auto reconstructed = qr.Q() * qr.R();

    for (std::size_t i = 0; i < 4; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            EXPECT_NEAR(reconstructed.at(i, j), a.at(i, j), 1e-5f);
}

TEST_F(TestQrDecomposition, Q_columns_are_orthonormal)
{
    math::Matrix<float, 4, 3> a{
        { 1.0f, 2.0f, 3.0f },
        { 4.0f, 5.0f, 6.0f },
        { 7.0f, 8.0f, 10.0f },
        { 1.0f, 0.0f, 2.0f }
    };

    qr.Decompose(a);
    auto qMat = qr.Q();
    auto qtq = qMat.Transpose() * qMat;

    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            EXPECT_NEAR(qtq.at(i, j), (i == j) ? 1.0f : 0.0f, 1e-5f);
}

TEST_F(TestQrDecomposition, R_is_upper_triangular)
{
    math::Matrix<float, 4, 3> a{
        { 1.0f, 2.0f, 3.0f },
        { 4.0f, 5.0f, 6.0f },
        { 7.0f, 8.0f, 10.0f },
        { 1.0f, 0.0f, 2.0f }
    };

    qr.Decompose(a);
    auto r = qr.R();

    for (std::size_t i = 1; i < 3; ++i)
        for (std::size_t j = 0; j < i; ++j)
            EXPECT_NEAR(r.at(i, j), 0.0f, 1e-5f);
}

TEST_F(TestQrDecomposition, solves_overdetermined_least_squares)
{
    math::Matrix<float, 4, 3> a{
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f },
        { 1.0f, 1.0f, 1.0f }
    };

    math::Vector<float, 4> b{ { 1.0f }, { 2.0f }, { 3.0f }, { 6.5f } };

    qr.Decompose(a);
    auto x = qr.SolveLeastSquares(b);

    EXPECT_NEAR(x.at(0, 0), 1.125f, 1e-4f);
    EXPECT_NEAR(x.at(1, 0), 2.125f, 1e-4f);
    EXPECT_NEAR(x.at(2, 0), 3.125f, 1e-4f);
}

TEST_F(TestQrDecomposition, matches_known_3x3)
{
    math::Matrix<float, 3, 3> a{
        { 12.0f, -51.0f, 4.0f },
        { 6.0f, 167.0f, -68.0f },
        { -4.0f, 24.0f, -41.0f }
    };

    qrSquare.Decompose(a);
    auto r = qrSquare.R();

    EXPECT_NEAR(std::abs(r.at(0, 0)), 14.0f, 1e-3f);
    EXPECT_NEAR(std::abs(r.at(1, 1)), 175.0f, 1e-3f);
    EXPECT_NEAR(std::abs(r.at(2, 2)), 35.0f, 1e-3f);
    EXPECT_NEAR(r.at(1, 0), 0.0f, 1e-3f);
    EXPECT_NEAR(r.at(2, 0), 0.0f, 1e-3f);
    EXPECT_NEAR(r.at(2, 1), 0.0f, 1e-3f);
}

TEST_F(TestQrDecomposition, apply_qtranspose_equals_explicit)
{
    math::Matrix<float, 4, 3> a{
        { 1.0f, 2.0f, 3.0f },
        { 4.0f, 5.0f, 6.0f },
        { 7.0f, 8.0f, 10.0f },
        { 1.0f, 0.0f, 2.0f }
    };

    qr.Decompose(a);

    math::Vector<float, 4> b{ { 1.0f }, { 2.0f }, { 3.0f }, { 4.0f } };
    math::Vector<float, 4> bCopy = b;

    qr.ApplyQtranspose(bCopy);

    auto qMat = qr.Q();
    auto explicit_qt_b = qMat.Transpose() * b;

    for (std::size_t i = 0; i < 3; ++i)
        EXPECT_NEAR(bCopy.at(i, 0), explicit_qt_b.at(i, 0), 1e-5f);
}

TEST_F(TestQrDecomposition, rank_deficient_returns_false)
{
    math::Matrix<float, 4, 3> a{
        { 1.0f, 2.0f, 3.0f },
        { 2.0f, 4.0f, 6.0f },
        { 3.0f, 6.0f, 9.0f },
        { 4.0f, 8.0f, 12.0f }
    };

    bool ok = qr.Decompose(a);

    EXPECT_FALSE(ok);
}

TEST_F(TestQrDecomposition, givens_update_adds_row)
{
    math::Matrix<float, 3, 3> a3{
        { 1.0f, 2.0f, 3.0f },
        { 4.0f, 5.0f, 6.0f },
        { 7.0f, 8.0f, 10.0f }
    };

    math::Matrix<float, 1, 3> newRow{
        { 1.0f, 0.0f, 2.0f }
    };

    math::Matrix<float, 4, 3> a4{
        { 1.0f, 2.0f, 3.0f },
        { 4.0f, 5.0f, 6.0f },
        { 7.0f, 8.0f, 10.0f },
        { 1.0f, 0.0f, 2.0f }
    };

    qrSquare.Decompose(a3);
    qrSquare.GivensUpdateRow(newRow);
    auto rUpdated = qrSquare.R();

    qr.Decompose(a4);
    auto rFull = qr.R();

    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = i; j < 3; ++j)
            EXPECT_NEAR(std::abs(rUpdated.at(i, j)), std::abs(rFull.at(i, j)), 1e-3f);
}

TEST_F(TestQrDecomposition, identity_factors_to_identity)
{
    auto a = math::SquareMatrix<float, 3>::Identity();

    qrSquare.Decompose(a);
    auto q = qrSquare.Q();
    auto r = qrSquare.R();

    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            EXPECT_NEAR(q.at(i, j), a.at(i, j), math::Tolerance<float>());

    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            EXPECT_NEAR(r.at(i, j), a.at(i, j), math::Tolerance<float>());
}

TEST_F(TestQrDecomposition, decompose_returns_true_on_full_rank)
{
    math::Matrix<float, 4, 3> a{
        { 1.0f, 2.0f, 3.0f },
        { 4.0f, 5.0f, 6.0f },
        { 7.0f, 8.0f, 10.0f },
        { 1.0f, 0.0f, 2.0f }
    };

    bool ok = qr.Decompose(a);

    EXPECT_TRUE(ok);
}

TEST_F(TestQrDecomposition, solves_square_system_exactly)
{
    math::Matrix<float, 3, 3> a{
        { 2.0f, 1.0f, 0.0f },
        { 1.0f, 3.0f, 1.0f },
        { 0.0f, 1.0f, 2.0f }
    };

    math::Vector<float, 3> b{ { 5.0f }, { 10.0f }, { 7.0f } };

    qrSquare.Decompose(a);
    auto x = qrSquare.SolveLeastSquares(b);

    EXPECT_NEAR(x.at(0, 0), 1.5f, 1e-4f);
    EXPECT_NEAR(x.at(1, 0), 2.0f, 1e-4f);
    EXPECT_NEAR(x.at(2, 0), 2.5f, 1e-4f);
}

TEST_F(TestQrDecomposition, normal_equations_satisfied)
{
    math::Matrix<float, 4, 3> a{
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f },
        { 1.0f, 1.0f, 1.0f }
    };

    math::Vector<float, 4> b{ { 1.0f }, { 2.0f }, { 3.0f }, { 6.5f } };

    qr.Decompose(a);
    auto x = qr.SolveLeastSquares(b);

    auto residual = a * x;
    math::Vector<float, 4> r{};
    for (std::size_t i = 0; i < 4; ++i)
        r.at(i, 0) = residual.at(i, 0) - b.at(i, 0);

    auto normalEq = a.Transpose() * r;
    for (std::size_t i = 0; i < 3; ++i)
        EXPECT_NEAR(normalEq.at(i, 0), 0.0f, 1e-4f);
}

TEST_F(TestQrDecomposition, decompose_is_deterministic)
{
    math::Matrix<float, 4, 3> a{
        { 1.0f, 2.0f, 3.0f },
        { 4.0f, 5.0f, 6.0f },
        { 7.0f, 8.0f, 10.0f },
        { 1.0f, 0.0f, 2.0f }
    };

    solvers::QrDecomposition<float, 4, 3> qr2{};
    qr.Decompose(a);
    qr2.Decompose(a);

    auto q1 = qr.Q();
    auto q2 = qr2.Q();
    auto r1 = qr.R();
    auto r2 = qr2.R();

    for (std::size_t i = 0; i < 4; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            EXPECT_FLOAT_EQ(q1.at(i, j), q2.at(i, j));

    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            EXPECT_FLOAT_EQ(r1.at(i, j), r2.at(i, j));
}

TEST_F(TestQrDecomposition, hilbert_3x3_reconstruction_within_tolerance)
{
    math::Matrix<float, 3, 3> h{
        { 1.0f, 0.5f, 1.0f / 3.0f },
        { 0.5f, 1.0f / 3.0f, 0.25f },
        { 1.0f / 3.0f, 0.25f, 0.2f }
    };

    bool ok = qrSquare.Decompose(h);
    EXPECT_TRUE(ok);

    auto reconstructed = qrSquare.Q() * qrSquare.R();
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            EXPECT_NEAR(reconstructed.at(i, j), h.at(i, j), 1e-4f);
}
