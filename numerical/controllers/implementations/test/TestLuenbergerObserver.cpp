#include "numerical/controllers/implementations/LuenbergerObserver.hpp"
#include "numerical/math/LinearTimeInvariant.hpp"
#include "numerical/math/Tolerance.hpp"
#include <array>
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    using Plant21 = math::LinearTimeInvariant<float, 2, 1, 1>;
    using Observer = controllers::LuenbergerObserver<float, 2, 1, 1>;

    static Plant21 MakeDoubleIntegrator()
    {
        Plant21 p{};
        p.A = math::SquareMatrix<float, 2>{ { 1.0f, 1.0f }, { 0.0f, 1.0f } };
        p.B = math::Matrix<float, 2, 1>{ { 0.0f }, { 1.0f } };
        p.C = math::Matrix<float, 1, 2>{ { 1.0f, 0.0f } };
        return p;
    }

    static math::Matrix<float, 2, 1> DesignGain()
    {
        std::array<float, 2> poles{ 0.2f, 0.3f };
        return Observer::AckermannGain(MakeDoubleIntegrator(), poles);
    }

    class TestLuenbergerObserver : public ::testing::Test
    {
    protected:
        Plant21 plant{ MakeDoubleIntegrator() };
        Observer observer{ plant, DesignGain() };
    };
}

TEST_F(TestLuenbergerObserver, converges_to_true_state)
{
    math::Vector<float, 2> xtrue{ { 1.0f }, { 0.5f } };
    observer.Reset(math::Vector<float, 2>{ { 0.0f }, { 0.0f } });
    math::Vector<float, 1> u{ { 0.0f } };

    for (int k = 0; k < 30; ++k)
    {
        math::Vector<float, 1> y = plant.C * xtrue;
        observer.Update(u, y);
        xtrue = plant.Step(xtrue, u);
    }

    auto xhat = observer.Estimate();
    EXPECT_NEAR(xhat.at(0, 0), xtrue.at(0, 0), 1e-2f);
    EXPECT_NEAR(xhat.at(1, 0), xtrue.at(1, 0), 1e-2f);
}

TEST_F(TestLuenbergerObserver, zero_innovation_when_estimate_correct)
{
    math::Vector<float, 2> x{ { 2.0f }, { -1.0f } };
    observer.Reset(x);
    math::Vector<float, 1> u{ { 0.5f } };

    math::Vector<float, 1> y = plant.C * x + plant.D * u;
    auto xhat = observer.Update(u, y);

    math::Vector<float, 2> xExpected = plant.Step(x, u);
    EXPECT_NEAR(xhat.at(0, 0), xExpected.at(0, 0), math::Tolerance<float>());
    EXPECT_NEAR(xhat.at(1, 0), xExpected.at(1, 0), math::Tolerance<float>());
}

TEST_F(TestLuenbergerObserver, ackermann_places_requested_poles)
{
    std::array<float, 2> poles{ 0.2f, 0.3f };
    auto L = Observer::AckermannGain(plant, poles);

    math::SquareMatrix<float, 2> AminusLC = plant.A;
    for (std::size_t i = 0; i < 2; ++i)
        for (std::size_t j = 0; j < 1; ++j)
            for (std::size_t k = 0; k < 2; ++k)
                AminusLC.at(i, k) -= L.at(i, j) * plant.C.at(j, k);

    float trace = AminusLC.Trace();
    float det = AminusLC.at(0, 0) * AminusLC.at(1, 1) - AminusLC.at(0, 1) * AminusLC.at(1, 0);

    float expectedTrace = 0.2f + 0.3f;
    float expectedDet = 0.2f * 0.3f;

    EXPECT_NEAR(trace, expectedTrace, 1e-3f);
    EXPECT_NEAR(det, expectedDet, 1e-3f);
}

TEST_F(TestLuenbergerObserver, rejects_initial_error_geometrically)
{
    std::array<float, 2> poles{ 0.2f, 0.3f };
    auto L = Observer::AckermannGain(plant, poles);
    Observer obs{ plant, L };

    math::Vector<float, 2> xtrue{ { 0.0f }, { 0.0f } };
    math::Vector<float, 2> x0hat{ { 1.0f }, { 0.0f } };
    obs.Reset(x0hat);
    math::Vector<float, 1> u{ { 0.0f } };

    auto e0Vec = x0hat;
    float e0 = std::sqrt(e0Vec.at(0, 0) * e0Vec.at(0, 0) + e0Vec.at(1, 0) * e0Vec.at(1, 0));

    math::Vector<float, 1> y = plant.C * xtrue;
    obs.Update(u, y);

    auto e1Vec = obs.Estimate();
    float e1 = std::sqrt(e1Vec.at(0, 0) * e1Vec.at(0, 0) + e1Vec.at(1, 0) * e1Vec.at(1, 0));

    EXPECT_LT(e1, e0);

    for (int k = 0; k < 10; ++k)
    {
        y = plant.C * xtrue;
        obs.Update(u, y);
    }
    auto eFinalVec = obs.Estimate();
    float eFinal = std::sqrt(eFinalVec.at(0, 0) * eFinalVec.at(0, 0) + eFinalVec.at(1, 0) * eFinalVec.at(1, 0));
    EXPECT_LT(eFinal, 0.1f * e0);
}

TEST_F(TestLuenbergerObserver, reset_sets_estimate)
{
    math::Vector<float, 2> x0{ { 3.0f }, { -2.0f } };
    observer.Reset(x0);

    EXPECT_NEAR(observer.Estimate().at(0, 0), 3.0f, math::Tolerance<float>());
    EXPECT_NEAR(observer.Estimate().at(1, 0), -2.0f, math::Tolerance<float>());
}

TEST_F(TestLuenbergerObserver, tracks_under_input_excitation)
{
    math::Vector<float, 2> xtrue{ { 0.0f }, { 0.0f } };
    observer.Reset(math::Vector<float, 2>{ { 1.0f }, { 0.0f } });

    std::array<float, 5> inputs{ 0.5f, -0.3f, 0.1f, 0.2f, -0.4f };

    for (int k = 0; k < 30; ++k)
    {
        math::Vector<float, 1> u{ { inputs[static_cast<std::size_t>(k) % inputs.size()] } };
        math::Vector<float, 1> y = plant.C * xtrue;
        observer.Update(u, y);
        xtrue = plant.Step(xtrue, u);
    }

    auto xhat = observer.Estimate();
    EXPECT_NEAR(xhat.at(0, 0), xtrue.at(0, 0), 5e-2f);
    EXPECT_NEAR(xhat.at(1, 0), xtrue.at(1, 0), 5e-2f);
}

TEST_F(TestLuenbergerObserver, feedthrough_D_accounted_in_output)
{
    Plant21 plantWithD{};
    plantWithD.A = plant.A;
    plantWithD.B = plant.B;
    plantWithD.C = plant.C;
    plantWithD.D = math::Matrix<float, 1, 1>{ { 0.5f } };

    std::array<float, 2> poles{ 0.2f, 0.3f };
    auto L = Observer::AckermannGain(plantWithD, poles);
    Observer obsD{ plantWithD, L };

    math::Vector<float, 2> x{ { 1.0f }, { 0.5f } };
    obsD.Reset(x);
    math::Vector<float, 1> u{ { 2.0f } };

    math::Vector<float, 1> y = plantWithD.C * x + plantWithD.D * u;
    auto xhat = obsD.Update(u, y);

    math::Vector<float, 2> xNext = plantWithD.Step(x, u);
    EXPECT_NEAR(xhat.at(0, 0), xNext.at(0, 0), math::Tolerance<float>());
    EXPECT_NEAR(xhat.at(1, 0), xNext.at(1, 0), math::Tolerance<float>());
}

TEST_F(TestLuenbergerObserver, determinism_same_sequence_identical_outputs)
{
    std::array<float, 2> poles{ 0.2f, 0.3f };
    auto L = Observer::AckermannGain(plant, poles);

    Observer obs1{ plant, L };
    Observer obs2{ plant, L };
    math::Vector<float, 2> x0{ { 1.5f }, { -0.5f } };
    obs1.Reset(x0);
    obs2.Reset(x0);

    std::array<float, 6> uSeq{ 0.3f, -0.1f, 0.5f, 0.0f, -0.2f, 0.4f };
    std::array<float, 6> ySeq{ 1.0f, 0.8f, 0.6f, 0.4f, 0.2f, 0.1f };

    for (std::size_t k = 0; k < uSeq.size(); ++k)
    {
        math::Vector<float, 1> u{ { uSeq[k] } };
        math::Vector<float, 1> y{ { ySeq[k] } };
        auto r1 = obs1.Update(u, y);
        auto r2 = obs2.Update(u, y);
        EXPECT_FLOAT_EQ(r1.at(0, 0), r2.at(0, 0));
        EXPECT_FLOAT_EQ(r1.at(1, 0), r2.at(1, 0));
    }
}

TEST_F(TestLuenbergerObserver, two_instances_interleaved_do_not_interfere)
{
    std::array<float, 2> poles{ 0.2f, 0.3f };
    auto L = Observer::AckermannGain(plant, poles);

    Observer obsA{ plant, L };
    Observer obsB{ plant, L };
    math::Vector<float, 2> x0{ { 1.0f }, { 0.0f } };
    obsA.Reset(x0);
    obsB.Reset(x0);

    std::array<float, 4> uSeq{ 0.2f, -0.3f, 0.1f, 0.4f };
    std::array<float, 4> ySeq{ 0.9f, 0.7f, 0.5f, 0.3f };

    for (std::size_t k = 0; k < uSeq.size(); ++k)
    {
        math::Vector<float, 1> u{ { uSeq[k] } };
        math::Vector<float, 1> y{ { ySeq[k] } };
        auto rA = obsA.Update(u, y);
        auto rB = obsB.Update(u, y);
        EXPECT_FLOAT_EQ(rA.at(0, 0), rB.at(0, 0));
        EXPECT_FLOAT_EQ(rA.at(1, 0), rB.at(1, 0));
    }
}

TEST_F(TestLuenbergerObserver, zero_input_zero_measurement_zero_state_stays_zero)
{
    observer.Reset(math::Vector<float, 2>{ { 0.0f }, { 0.0f } });
    math::Vector<float, 1> u{ { 0.0f } };
    math::Vector<float, 1> y{ { 0.0f } };

    for (int k = 0; k < 10; ++k)
    {
        auto xhat = observer.Update(u, y);
        EXPECT_FLOAT_EQ(xhat.at(0, 0), 0.0f);
        EXPECT_FLOAT_EQ(xhat.at(1, 0), 0.0f);
    }
}

TEST_F(TestLuenbergerObserver, closed_loop_eigenvalues_inside_unit_disk)
{
    std::array<float, 2> poles{ 0.2f, 0.3f };
    auto L = Observer::AckermannGain(plant, poles);

    math::SquareMatrix<float, 2> AminusLC = plant.A;
    for (std::size_t i = 0; i < 2; ++i)
        for (std::size_t k = 0; k < 2; ++k)
            AminusLC.at(i, k) -= L.at(i, 0) * plant.C.at(0, k);

    float tr = AminusLC.Trace();
    float dt = AminusLC.at(0, 0) * AminusLC.at(1, 1) - AminusLC.at(0, 1) * AminusLC.at(1, 0);

    float discriminant = tr * tr - 4.0f * dt;
    float sqrtD = std::sqrt(std::abs(discriminant));
    float lambda1mag;
    float lambda2mag;

    if (discriminant >= 0.0f)
    {
        lambda1mag = std::abs(0.5f * (tr + sqrtD));
        lambda2mag = std::abs(0.5f * (tr - sqrtD));
    }
    else
    {
        float realPart = 0.5f * tr;
        float imagPart = 0.5f * sqrtD;
        lambda1mag = std::sqrt(realPart * realPart + imagPart * imagPart);
        lambda2mag = lambda1mag;
    }

    EXPECT_LT(lambda1mag, 1.0f);
    EXPECT_LT(lambda2mag, 1.0f);
}
