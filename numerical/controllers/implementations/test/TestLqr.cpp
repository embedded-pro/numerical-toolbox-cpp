#include "numerical/controllers/implementations/Lqr.hpp"
#include "numerical/math/LinearTimeInvariant.hpp"
#include "numerical/math/Tolerance.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    class TestLqr : public ::testing::Test
    {
    protected:
        static constexpr float kGoldenRatio = 1.6180340f;
        static constexpr float kGoldenK = 0.6180340f;
        static constexpr float kK0 = 7.604472f;
        static constexpr float kK1 = 4.977648f;
        static constexpr float kClosedLoopRadius = 0.760447f;

        math::SquareMatrix<float, 2> A2{
            { 1.0f, 0.1f },
            { 0.0f, 1.0f }
        };
        math::Matrix<float, 2, 1> B2{
            { 0.0f },
            { 0.1f }
        };
        math::SquareMatrix<float, 2> Q2{
            { 10.0f, 0.0f },
            { 0.0f, 1.0f }
        };
        math::SquareMatrix<float, 1> R2{ { 0.1f } };
    };
}

TEST_F(TestLqr, precomputed_gain_control_law_matches_negative_gain_times_state)
{
    math::Matrix<float, 1, 2> K{ { 0.5f, 0.3f } };
    controllers::Lqr<float, 2, 1> lqr{ K };

    math::Vector<float, 2> state{ { 1.0f }, { 2.0f } };
    auto u = lqr.ComputeControl(state);

    EXPECT_NEAR(u.at(0, 0), -(0.5f * 1.0f + 0.3f * 2.0f), math::Tolerance<float>());
}

TEST_F(TestLqr, zero_state_produces_zero_control)
{
    math::Matrix<float, 1, 2> K{ { 0.5f, 0.3f } };
    controllers::Lqr<float, 2, 1> lqr{ K };

    math::Vector<float, 2> zeroState{};
    auto u = lqr.ComputeControl(zeroState);

    EXPECT_NEAR(u.at(0, 0), 0.0f, math::Tolerance<float>());
}

TEST_F(TestLqr, multi_input_precomputed_gain_applies_correctly)
{
    math::Matrix<float, 2, 2> K{
        { 1.0f, 0.0f },
        { 0.0f, 2.0f }
    };
    controllers::Lqr<float, 2, 2> lqr{ K };

    math::Vector<float, 2> state{ { 3.0f }, { 4.0f } };
    auto u = lqr.ComputeControl(state);

    EXPECT_NEAR(u.at(0, 0), -3.0f, math::Tolerance<float>());
    EXPECT_NEAR(u.at(1, 0), -8.0f, math::Tolerance<float>());
}

TEST_F(TestLqr, scalar_dare_gain_matches_analytic_golden_ratio_root)
{
    math::SquareMatrix<float, 1> A{ { 1.0f } };
    math::Matrix<float, 1, 1> B{ { 1.0f } };
    math::SquareMatrix<float, 1> Q{ { 1.0f } };
    math::SquareMatrix<float, 1> R{ { 1.0f } };

    controllers::Lqr<float, 1, 1> lqr{ A, B, Q, R };

    EXPECT_NEAR(lqr.GetGain().at(0, 0), kGoldenK, 1e-4f);
}

TEST_F(TestLqr, scalar_dare_riccati_solution_matches_analytic_golden_ratio)
{
    math::SquareMatrix<float, 1> A{ { 1.0f } };
    math::Matrix<float, 1, 1> B{ { 1.0f } };
    math::SquareMatrix<float, 1> Q{ { 1.0f } };
    math::SquareMatrix<float, 1> R{ { 1.0f } };

    controllers::Lqr<float, 1, 1> lqr{ A, B, Q, R };

    EXPECT_NEAR(lqr.GetRiccatiSolution().at(0, 0), kGoldenRatio, 1e-4f);
}

TEST_F(TestLqr, double_integrator_dare_gain_matches_independent_reference)
{
    controllers::Lqr<float, 2, 1> lqr{ A2, B2, Q2, R2 };

    EXPECT_NEAR(lqr.GetGain().at(0, 0), kK0, 5e-2f);
    EXPECT_NEAR(lqr.GetGain().at(0, 1), kK1, 5e-2f);
}

TEST_F(TestLqr, double_integrator_closed_loop_spectral_radius_below_one)
{
    controllers::Lqr<float, 2, 1> lqr{ A2, B2, Q2, R2 };

    const float k0 = lqr.GetGain().at(0, 0);
    const float k1 = lqr.GetGain().at(0, 1);

    const float cl00 = 1.0f;
    const float cl01 = 0.1f;
    const float cl10 = -0.1f * k0;
    const float cl11 = 1.0f - 0.1f * k1;

    const float trace = cl00 + cl11;
    const float det = cl00 * cl11 - cl01 * cl10;
    const float disc = trace * trace - 4.0f * det;

    float spectralRadius{};
    if (disc >= 0.0f)
    {
        const float e1 = (trace + std::sqrt(disc)) / 2.0f;
        const float e2 = (trace - std::sqrt(disc)) / 2.0f;
        spectralRadius = std::max(std::abs(e1), std::abs(e2));
    }
    else
    {
        const float re = trace / 2.0f;
        const float im = std::sqrt(-disc) / 2.0f;
        spectralRadius = std::sqrt(re * re + im * im);
    }

    EXPECT_NEAR(spectralRadius, kClosedLoopRadius, 5e-3f);
    EXPECT_LT(spectralRadius, 1.0f);
}

TEST_F(TestLqr, lti_constructor_produces_identical_gain_to_matrix_constructor)
{
    auto plant = math::LinearTimeInvariant<float, 2, 1>::WithFullStateOutput(A2, B2);

    controllers::Lqr<float, 2, 1> lqrMat{ A2, B2, Q2, R2 };
    controllers::Lqr<float, 2, 1> lqrLti{ plant, Q2, R2 };

    EXPECT_NEAR(lqrMat.GetGain().at(0, 0), lqrLti.GetGain().at(0, 0), math::Tolerance<float>());
    EXPECT_NEAR(lqrMat.GetGain().at(0, 1), lqrLti.GetGain().at(0, 1), math::Tolerance<float>());
}

TEST_F(TestLqr, high_frequency_plant_converges_with_raised_iteration_cap)
{
    math::SquareMatrix<float, 2> A{
        { 1.0f, 0.001f },
        { 0.0f, 1.0f }
    };
    math::Matrix<float, 2, 1> B{
        { 0.0f },
        { 0.001f }
    };
    math::SquareMatrix<float, 2> Q{
        { 10.0f, 0.0f },
        { 0.0f, 1.0f }
    };
    math::SquareMatrix<float, 1> R{ { 0.1f } };

    controllers::Lqr<float, 2, 1, 30000> lqr{ A, B, Q, R };

    EXPECT_NEAR(lqr.GetGain().at(0, 0), 9.972051f, 0.1f);
    EXPECT_NEAR(lqr.GetGain().at(0, 1), 5.472012f, 0.1f);
    EXPECT_NEAR(lqr.GetRiccatiSolution().at(0, 0), 5487.013672f, 5.0f);
    EXPECT_NEAR(lqr.GetRiccatiSolution().at(1, 1), 549.203735f, 1.0f);
}

TEST_F(TestLqr, two_instances_with_same_matrices_produce_identical_gains)
{
    controllers::Lqr<float, 2, 1> lqr1{ A2, B2, Q2, R2 };
    controllers::Lqr<float, 2, 1> lqr2{ A2, B2, Q2, R2 };

    EXPECT_FLOAT_EQ(lqr1.GetGain().at(0, 0), lqr2.GetGain().at(0, 0));
    EXPECT_FLOAT_EQ(lqr1.GetGain().at(0, 1), lqr2.GetGain().at(0, 1));
}

TEST_F(TestLqr, closed_loop_state_norm_decays_over_simulation)
{
    controllers::Lqr<float, 2, 1> lqr{ A2, B2, Q2, R2 };

    math::Vector<float, 2> x{ { 1.0f }, { 0.0f } };
    const float normInit = std::sqrt(x.at(0, 0) * x.at(0, 0) + x.at(1, 0) * x.at(1, 0));

    for (std::size_t step = 0; step < 100; ++step)
    {
        const auto u = lqr.ComputeControl(x);
        const float x0 = A2.at(0, 0) * x.at(0, 0) + A2.at(0, 1) * x.at(1, 0) + B2.at(0, 0) * u.at(0, 0);
        const float x1 = A2.at(1, 0) * x.at(0, 0) + A2.at(1, 1) * x.at(1, 0) + B2.at(1, 0) * u.at(0, 0);
        x = math::Vector<float, 2>{ { x0 }, { x1 } };
    }

    const float normFinal = std::sqrt(x.at(0, 0) * x.at(0, 0) + x.at(1, 0) * x.at(1, 0));
    EXPECT_LT(normFinal, normInit * 1e-4f);
}
