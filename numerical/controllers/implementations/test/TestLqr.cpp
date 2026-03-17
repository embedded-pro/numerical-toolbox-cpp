#include "numerical/controllers/implementations/Lqr.hpp"
#include "numerical/math/QNumber.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    class TestLqr : public ::testing::Test
    {
    protected:
        static bool AreNear(float a, float b, float epsilon = 1e-3f)
        {
            return std::abs(a - b) < epsilon;
        }

        template<typename T, std::size_t Rows, std::size_t Cols>
        static bool AreMatricesNear(const math::Matrix<T, Rows, Cols>& a,
            const math::Matrix<T, Rows, Cols>& b, float epsilon = 1e-3f)
        {
            for (std::size_t i = 0; i < Rows; ++i)
            {
                for (std::size_t j = 0; j < Cols; ++j)
                {
                    float va, vb;
                    if constexpr (std::is_same_v<T, float>)
                    {
                        va = a.at(i, j);
                        vb = b.at(i, j);
                    }
                    else
                    {
                        va = a.at(i, j).ToFloat();
                        vb = b.at(i, j).ToFloat();
                    }

                    if (std::abs(va - vb) > epsilon)
                        return false;
                }
            }
            return true;
        }
    };
}

TEST_F(TestLqr, scalar_system_computes_correct_gain)
{
    math::SquareMatrix<float, 1> A{ { 1.0f } };
    math::Matrix<float, 1, 1> B{ { 1.0f } };
    math::SquareMatrix<float, 1> Q{ { 1.0f } };
    math::SquareMatrix<float, 1> R{ { 1.0f } };

    controllers::Lqr<float, 1, 1> lqr(A, B, Q, R);

    float goldenRatio = (1.0f + std::sqrt(5.0f)) / 2.0f;
    float expectedGain = goldenRatio / (1.0f + goldenRatio);
    EXPECT_NEAR(lqr.GetGain().at(0, 0), expectedGain, 1e-3f);
}

TEST_F(TestLqr, zero_state_produces_zero_control)
{
    math::SquareMatrix<float, 2> A{
        { 1.0f, 1.0f },
        { 0.0f, 1.0f }
    };

    math::Matrix<float, 2, 1> B{
        { 0.5f },
        { 1.0f }
    };

    math::SquareMatrix<float, 2> Q{
        { 1.0f, 0.0f },
        { 0.0f, 1.0f }
    };

    math::SquareMatrix<float, 1> R{ { 1.0f } };

    controllers::Lqr<float, 2, 1> lqr(A, B, Q, R);

    math::Vector<float, 2> zeroState{ { 0.0f }, { 0.0f } };
    auto u = lqr.ComputeControl(zeroState);

    EXPECT_NEAR(u.at(0, 0), 0.0f, 1e-6f);
}

TEST_F(TestLqr, control_opposes_positive_state)
{
    math::SquareMatrix<float, 1> A{ { 1.0f } };
    math::Matrix<float, 1, 1> B{ { 1.0f } };
    math::SquareMatrix<float, 1> Q{ { 1.0f } };
    math::SquareMatrix<float, 1> R{ { 1.0f } };

    controllers::Lqr<float, 1, 1> lqr(A, B, Q, R);

    math::Vector<float, 1> positiveState{ { 1.0f } };
    auto u = lqr.ComputeControl(positiveState);

    EXPECT_LT(u.at(0, 0), 0.0f);
}

TEST_F(TestLqr, control_opposes_negative_state)
{
    math::SquareMatrix<float, 1> A{ { 1.0f } };
    math::Matrix<float, 1, 1> B{ { 1.0f } };
    math::SquareMatrix<float, 1> Q{ { 1.0f } };
    math::SquareMatrix<float, 1> R{ { 1.0f } };

    controllers::Lqr<float, 1, 1> lqr(A, B, Q, R);

    math::Vector<float, 1> negativeState{ { -1.0f } };
    auto u = lqr.ComputeControl(negativeState);

    EXPECT_GT(u.at(0, 0), 0.0f);
}

TEST_F(TestLqr, precomputed_gain_constructor)
{
    math::Matrix<float, 1, 2> K{
        { 0.5f, 0.3f }
    };

    controllers::Lqr<float, 2, 1> lqr(K);

    math::Vector<float, 2> state{ { 1.0f }, { 2.0f } };
    auto u = lqr.ComputeControl(state);

    EXPECT_NEAR(u.at(0, 0), -1.1f, 1e-3f);
}

TEST_F(TestLqr, double_integrator_gain_is_stabilizing)
{
    math::SquareMatrix<float, 2> A{
        { 1.0f, 1.0f },
        { 0.0f, 1.0f }
    };

    math::Matrix<float, 2, 1> B{
        { 0.5f },
        { 1.0f }
    };

    math::SquareMatrix<float, 2> Q{
        { 1.0f, 0.0f },
        { 0.0f, 1.0f }
    };

    math::SquareMatrix<float, 1> R{ { 1.0f } };

    controllers::Lqr<float, 2, 1> lqr(A, B, Q, R);

    math::Vector<float, 2> state{ { 1.0f }, { 0.0f } };

    float initialNorm = std::abs(state.at(0, 0)) + std::abs(state.at(1, 0));

    for (int step = 0; step < 20; ++step)
    {
        auto u = lqr.ComputeControl(state);

        auto nextState = A * state + B * u;
        state = nextState;
    }

    float finalNorm = std::abs(state.at(0, 0)) + std::abs(state.at(1, 0));

    EXPECT_LT(finalNorm, initialNorm * 0.01f);
}

TEST_F(TestLqr, riccati_solution_is_symmetric)
{
    math::SquareMatrix<float, 2> A{
        { 1.0f, 1.0f },
        { 0.0f, 1.0f }
    };

    math::Matrix<float, 2, 1> B{
        { 0.5f },
        { 1.0f }
    };

    math::SquareMatrix<float, 2> Q{
        { 1.0f, 0.0f },
        { 0.0f, 1.0f }
    };

    math::SquareMatrix<float, 1> R{ { 1.0f } };

    controllers::Lqr<float, 2, 1> lqr(A, B, Q, R);
    auto P = lqr.GetRiccatiSolution();

    EXPECT_NEAR(P.at(0, 1), P.at(1, 0), 1e-3f);
}

TEST_F(TestLqr, higher_Q_increases_gain)
{
    math::SquareMatrix<float, 1> A{ { 1.0f } };
    math::Matrix<float, 1, 1> B{ { 1.0f } };
    math::SquareMatrix<float, 1> R{ { 1.0f } };

    math::SquareMatrix<float, 1> Q_low{ { 0.5f } };
    math::SquareMatrix<float, 1> Q_high{ { 5.0f } };

    controllers::Lqr<float, 1, 1> lqr_low(A, B, Q_low, R);
    controllers::Lqr<float, 1, 1> lqr_high(A, B, Q_high, R);

    EXPECT_GT(lqr_high.GetGain().at(0, 0), lqr_low.GetGain().at(0, 0));
}

TEST_F(TestLqr, higher_R_decreases_gain)
{
    math::SquareMatrix<float, 1> A{ { 1.0f } };
    math::Matrix<float, 1, 1> B{ { 1.0f } };
    math::SquareMatrix<float, 1> Q{ { 1.0f } };

    math::SquareMatrix<float, 1> R_low{ { 0.5f } };
    math::SquareMatrix<float, 1> R_high{ { 5.0f } };

    controllers::Lqr<float, 1, 1> lqr_low(A, B, Q, R_low);
    controllers::Lqr<float, 1, 1> lqr_high(A, B, Q, R_high);

    EXPECT_LT(lqr_high.GetGain().at(0, 0), lqr_low.GetGain().at(0, 0));
}
