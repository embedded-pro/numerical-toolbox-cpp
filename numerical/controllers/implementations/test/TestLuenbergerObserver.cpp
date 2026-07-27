#include "numerical/controllers/implementations/LuenbergerObserver.hpp"
#include "numerical/math/LinearTimeInvariant.hpp"
#include "numerical/math/Tolerance.hpp"
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

TEST_F(TestLuenbergerObserver, faster_poles_converge_quicker)
{
    math::Vector<float, 2> xtrue{ { 0.0f }, { 0.0f } };
    math::Vector<float, 2> xhatWrong{ { 1.0f }, { 1.0f } };
    math::Vector<float, 1> u{ { 0.0f } };

    std::array<float, 2> slowPoles{ 0.5f, 0.5f };
    std::array<float, 2> fastPoles{ 0.05f, 0.05f };

    auto Lslow = Observer::AckermannGain(plant, slowPoles);
    auto Lfast = Observer::AckermannGain(plant, fastPoles);

    Observer slowObs{ plant, Lslow };
    Observer fastObs{ plant, Lfast };
    slowObs.Reset(xhatWrong);
    fastObs.Reset(xhatWrong);

    constexpr float tol = 0.05f;
    int slowSteps = 0;
    int fastSteps = 0;

    for (int k = 0; k < 100; ++k)
    {
        math::Vector<float, 1> y = plant.C * xtrue;

        slowObs.Update(u, y);
        fastObs.Update(u, y);

        auto eS0 = slowObs.Estimate().at(0, 0) - xtrue.at(0, 0);
        auto eS1 = slowObs.Estimate().at(1, 0) - xtrue.at(1, 0);
        auto eF0 = fastObs.Estimate().at(0, 0) - xtrue.at(0, 0);
        auto eF1 = fastObs.Estimate().at(1, 0) - xtrue.at(1, 0);

        float normSlow = std::sqrt(eS0 * eS0 + eS1 * eS1);
        float normFast = std::sqrt(eF0 * eF0 + eF1 * eF1);

        if (slowSteps == 0 && normSlow < tol)
            slowSteps = k + 1;
        if (fastSteps == 0 && normFast < tol)
            fastSteps = k + 1;
    }

    EXPECT_LT(fastSteps, slowSteps);
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
