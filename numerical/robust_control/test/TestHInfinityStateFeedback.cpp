#include "numerical/controllers/implementations/Lqr.hpp"
#include "numerical/math/Matrix.hpp"
#include "numerical/math/Tolerance.hpp"
#include "numerical/robust_control/HInfinityStateFeedback.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    using Plant = robust_control::GeneralizedPlant<float, 2, 1, 1, 1>;
    using HInf = robust_control::HInfinityStateFeedback<float, 2, 1, 1, 1>;

    Plant MakeGeneralizedPlant()
    {
        Plant p{};
        p.A.at(0, 0) = 1.0f;
        p.A.at(0, 1) = 0.1f;
        p.A.at(1, 0) = 0.0f;
        p.A.at(1, 1) = 0.9f;

        p.B1.at(0, 0) = 0.0f;
        p.B1.at(1, 0) = 0.1f;

        p.B2.at(0, 0) = 0.0f;
        p.B2.at(1, 0) = 1.0f;

        p.C1.at(0, 0) = 1.0f;
        p.C1.at(0, 1) = 0.0f;

        p.D12.at(0, 0) = 0.0f;
        return p;
    }

    class TestHInfinityStateFeedback : public ::testing::Test
    {
    protected:
        Plant plant{ MakeGeneralizedPlant() };
        HInf hinf{ plant };
    };
}

TEST_F(TestHInfinityStateFeedback, reduces_to_lqr_as_gamma_large)
{
    const bool ok = hinf.Synthesize(0.5f, 1000.0f, 1e-3f);
    EXPECT_TRUE(ok);

    math::SquareMatrix<float, 2> Q{ plant.C1.Transpose() * plant.C1 };
    math::SquareMatrix<float, 1> R{};
    R.at(0, 0) = 1.0f;
    controllers::Lqr<float, 2, 1> lqr{ plant.A, plant.B2, Q, R };

    const auto& Khinf = hinf.Gain();
    const auto& Klqr = lqr.GetGain();

    for (std::size_t c = 0; c < 2; ++c)
        EXPECT_NEAR(Khinf.at(0, c), Klqr.at(0, c), 5e-2f);
}

TEST_F(TestHInfinityStateFeedback, closed_loop_is_schur_stable)
{
    const bool ok = hinf.Synthesize(0.5f, 100.0f, 1e-3f);
    EXPECT_TRUE(ok);

    const auto& K = hinf.Gain();
    auto clA = plant.A - plant.B2 * K;

    std::array<float, 3> charPoly{};
    charPoly[0] = 1.0f;
    charPoly[1] = -(clA.at(0, 0) + clA.at(1, 1));
    charPoly[2] = clA.at(0, 0) * clA.at(1, 1) - clA.at(0, 1) * clA.at(1, 0);

    solvers::DurandKerner<float, 2> dk{};
    auto roots = dk.Solve(std::span<const float>{ charPoly.data(), 3 });

    for (const auto& root : roots)
        EXPECT_LT(std::abs(root), 1.0f);
}

TEST_F(TestHInfinityStateFeedback, achieves_target_gamma)
{
    const float gTarget = 5.0f;
    const bool ok = hinf.Synthesize(0.5f, gTarget * 2.0f, 1e-3f);
    EXPECT_TRUE(ok);

    const auto& K = hinf.Gain();
    auto clA = plant.A - plant.B2 * K;
    auto clB = plant.B1;
    const auto& C1 = plant.C1;

    float outputEnergy{ 0.0f };
    float inputEnergy{ 0.0f };
    math::Vector<float, 2> x{};

    const float wMag = 1.0f;
    for (int step = 0; step < 200; ++step)
    {
        math::Vector<float, 1> w{};
        w.at(0, 0) = (step % 2 == 0) ? wMag : -wMag;

        auto z = C1 * x;
        outputEnergy += z.at(0, 0) * z.at(0, 0);
        inputEnergy += w.at(0, 0) * w.at(0, 0);

        x = clA * x + clB * w;
    }

    if (inputEnergy > 0.0f)
        EXPECT_LE(outputEnergy / inputEnergy, hinf.Gamma() * hinf.Gamma() + 1.0f);
}

TEST_F(TestHInfinityStateFeedback, bisection_finds_minimal_gamma)
{
    const bool ok = hinf.Synthesize(0.01f, 20.0f, 1e-2f);
    EXPECT_TRUE(ok);
    EXPECT_LT(hinf.Gamma(), 20.0f);
    EXPECT_GT(hinf.Gamma(), 0.01f);
}

TEST_F(TestHInfinityStateFeedback, infeasible_below_gamma_optimum)
{
    const bool ok = hinf.Synthesize(0.01f, 20.0f, 1e-2f);
    EXPECT_TRUE(ok);

    const float gammaStar = hinf.Gamma();
    HInf hinfLow{ plant };
    const bool okLow = hinfLow.Synthesize(0.0f, gammaStar * 0.5f, 1e-3f);
    EXPECT_FALSE(okLow);
}

TEST_F(TestHInfinityStateFeedback, gain_matches_gare_solution)
{
    const bool ok = hinf.Synthesize(0.5f, 100.0f, 1e-3f);
    EXPECT_TRUE(ok);

    constexpr std::size_t AugInputSize = 2;
    math::Matrix<float, 2, AugInputSize> B{};
    B.at(0, 0) = plant.B2.at(0, 0);
    B.at(1, 0) = plant.B2.at(1, 0);
    B.at(0, 1) = plant.B1.at(0, 0);
    B.at(1, 1) = plant.B1.at(1, 0);

    const float g = hinf.Gamma();
    math::SquareMatrix<float, AugInputSize> Rtilde{};
    Rtilde.at(0, 0) = 1.0f;
    Rtilde.at(1, 1) = -(g * g);

    auto Q = plant.C1.Transpose() * plant.C1;
    solvers::DiscreteAlgebraicRiccatiEquation<float, 2, AugInputSize> dare{};
    auto Xref = dare.Solve(plant.A, B, Q, Rtilde);

    auto BtX = B.Transpose() * Xref;
    auto S = Rtilde + BtX * B;
    auto BtXA = BtX * plant.A;
    auto Kfull = solvers::SolveSystem<float, AugInputSize, 2>(S, BtXA);

    const auto& K = hinf.Gain();
    EXPECT_NEAR(K.at(0, 0), Kfull.at(0, 0), 1e-3f);
    EXPECT_NEAR(K.at(0, 1), Kfull.at(0, 1), 1e-3f);
}

TEST_F(TestHInfinityStateFeedback, rejects_worst_case_disturbance)
{
    const bool ok = hinf.Synthesize(0.5f, 100.0f, 1e-3f);
    EXPECT_TRUE(ok);

    const float g = hinf.Gamma();
    const auto& K = hinf.Gain();
    auto clA = plant.A - plant.B2 * K;
    const auto& C1 = plant.C1;

    math::Vector<float, 2> x{};
    x.at(0, 0) = 1.0f;

    float outputEnergy{ 0.0f };
    float inputEnergy{ 0.0f };
    const float wMag = 0.5f;

    for (int step = 0; step < 100; ++step)
    {
        math::Vector<float, 1> w{};
        w.at(0, 0) = (step % 2 == 0) ? wMag : -wMag;

        auto z = C1 * x;
        outputEnergy += z.at(0, 0) * z.at(0, 0);
        inputEnergy += w.at(0, 0) * w.at(0, 0);

        x = clA * x + plant.B1 * w;
    }

    EXPECT_LE(outputEnergy, g * g * inputEnergy + 10.0f);
}

TEST_F(TestHInfinityStateFeedback, compute_control_is_negative_feedback)
{
    const bool ok = hinf.Synthesize(0.5f, 100.0f, 1e-3f);
    EXPECT_TRUE(ok);

    math::Vector<float, 2> x{};
    x.at(0, 0) = 1.0f;
    x.at(1, 0) = 0.5f;

    const auto u = hinf.ComputeControl(x);
    const auto& K = hinf.Gain();

    const float expected = -(K.at(0, 0) * x.at(0, 0) + K.at(0, 1) * x.at(1, 0));
    EXPECT_NEAR(u.at(0, 0), expected, math::Tolerance<float>());
}
