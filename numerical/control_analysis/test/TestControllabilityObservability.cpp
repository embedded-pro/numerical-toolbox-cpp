#include "numerical/control_analysis/ControllabilityObservability.hpp"
#include <cmath>
#include <gtest/gtest.h>

namespace
{
    using CO = control_analysis::ControllabilityObservability<float, 2, 1, 1>;
    using LTI = math::LinearTimeInvariant<float, 2, 1, 1>;

    class TestControllabilityObservability : public ::testing::Test
    {
    protected:
        LTI plant;

        void SetUp() override
        {
            plant.A = math::Matrix<float, 2, 2>{
                { 0.0f, 1.0f },
                { -2.0f, -3.0f }
            };
            plant.B = math::Matrix<float, 2, 1>{
                { 0.0f },
                { 1.0f }
            };
            plant.C = math::Matrix<float, 1, 2>{
                { 1.0f, 0.0f }
            };
            plant.D = math::Matrix<float, 1, 1>{
                { 0.0f }
            };
        }
    };
}

TEST_F(TestControllabilityObservability, controllability_matrix_matches_hand_value)
{
    auto Wc = CO::ControllabilityMatrix(plant);

    EXPECT_NEAR(Wc.at(0, 0), 0.0f, 1e-5f);
    EXPECT_NEAR(Wc.at(0, 1), 1.0f, 1e-5f);
    EXPECT_NEAR(Wc.at(1, 0), 1.0f, 1e-5f);
    EXPECT_NEAR(Wc.at(1, 1), -3.0f, 1e-5f);
}

TEST_F(TestControllabilityObservability, observability_matrix_matches_hand_value)
{
    auto Wo = CO::ObservabilityMatrix(plant);

    EXPECT_NEAR(Wo.at(0, 0), 1.0f, 1e-5f);
    EXPECT_NEAR(Wo.at(0, 1), 0.0f, 1e-5f);
    EXPECT_NEAR(Wo.at(1, 0), 0.0f, 1e-5f);
    EXPECT_NEAR(Wo.at(1, 1), 1.0f, 1e-5f);
}

TEST_F(TestControllabilityObservability, controllable_pair_has_full_rank)
{
    auto Wc = CO::ControllabilityMatrix(plant);
    std::size_t rank = CO::Rank(Wc, 1e-6f);

    EXPECT_EQ(rank, 2u);
    EXPECT_TRUE(CO::IsControllable(plant));
}

TEST_F(TestControllabilityObservability, uncontrollable_pair_detected)
{
    LTI uncontrollablePlant;
    uncontrollablePlant.A = math::Matrix<float, 2, 2>{
        { 0.5f, 0.0f },
        { 0.0f, 0.5f }
    };
    uncontrollablePlant.B = math::Matrix<float, 2, 1>{
        { 1.0f },
        { 0.0f }
    };
    uncontrollablePlant.C = math::Matrix<float, 1, 2>{
        { 1.0f, 0.0f }
    };
    uncontrollablePlant.D = math::Matrix<float, 1, 1>{
        { 0.0f }
    };

    auto Wc = CO::ControllabilityMatrix(uncontrollablePlant);
    std::size_t rank = CO::Rank(Wc, 1e-6f);

    EXPECT_EQ(rank, 1u);
    EXPECT_FALSE(CO::IsControllable(uncontrollablePlant));
}

TEST_F(TestControllabilityObservability, observable_pair_has_full_rank)
{
    EXPECT_TRUE(CO::IsObservable(plant));
}

TEST_F(TestControllabilityObservability, unobservable_pair_detected)
{
    LTI unobservablePlant;
    unobservablePlant.A = math::Matrix<float, 2, 2>{
        { 0.5f, 0.0f },
        { 0.0f, 0.5f }
    };
    unobservablePlant.B = math::Matrix<float, 2, 1>{
        { 0.0f },
        { 1.0f }
    };
    unobservablePlant.C = math::Matrix<float, 1, 2>{
        { 1.0f, 0.0f }
    };
    unobservablePlant.D = math::Matrix<float, 1, 1>{
        { 0.0f }
    };

    EXPECT_FALSE(CO::IsObservable(unobservablePlant));
}

TEST_F(TestControllabilityObservability, duality_ctrb_equals_obsv_of_transpose)
{
    LTI dualPlant;
    dualPlant.A = plant.A.Transpose();
    dualPlant.B = plant.C.Transpose();
    dualPlant.C = plant.B.Transpose();
    dualPlant.D = plant.D.Transpose();

    auto Wc_dual = CO::ControllabilityMatrix(dualPlant);
    auto Wo = CO::ObservabilityMatrix(plant);
    auto Wo_T = Wo.Transpose();

    for (std::size_t r = 0; r < 2; ++r)
        for (std::size_t c = 0; c < 2; ++c)
            EXPECT_NEAR(Wc_dual.at(r, c), Wo_T.at(r, c), 1e-5f);
}

TEST_F(TestControllabilityObservability, gramian_solves_discrete_lyapunov)
{
    LTI stablePlant;
    stablePlant.A = math::Matrix<float, 2, 2>{
        { 0.0f, 1.0f },
        { -0.2f, -0.3f }
    };
    stablePlant.B = math::Matrix<float, 2, 1>{
        { 0.0f },
        { 1.0f }
    };
    stablePlant.C = math::Matrix<float, 1, 2>{
        { 1.0f, 0.0f }
    };
    stablePlant.D = math::Matrix<float, 1, 1>{
        { 0.0f }
    };

    auto Wc = CO::ControllabilityGramian(stablePlant);
    auto BBt = stablePlant.B * stablePlant.B.Transpose();
    auto residual = stablePlant.A * Wc * stablePlant.A.Transpose() - Wc + BBt;

    float maxResidual{ 0.0f };
    for (std::size_t r = 0; r < 2; ++r)
        for (std::size_t c = 0; c < 2; ++c)
        {
            float v = residual.at(r, c);
            if (v < 0.0f)
                v = -v;
            if (v > maxResidual)
                maxResidual = v;
        }

    EXPECT_NEAR(maxResidual, 0.0f, 1e-3f);
}

TEST_F(TestControllabilityObservability, observability_gramian_solves_discrete_lyapunov)
{
    LTI stablePlant;
    stablePlant.A = math::Matrix<float, 2, 2>{
        { 0.0f, 1.0f },
        { -0.2f, -0.3f }
    };
    stablePlant.B = math::Matrix<float, 2, 1>{
        { 0.0f },
        { 1.0f }
    };
    stablePlant.C = math::Matrix<float, 1, 2>{
        { 1.0f, 0.0f }
    };
    stablePlant.D = math::Matrix<float, 1, 1>{
        { 0.0f }
    };

    auto Wo = CO::ObservabilityGramian(stablePlant);
    auto CtC = stablePlant.C.Transpose() * stablePlant.C;
    auto residual = stablePlant.A.Transpose() * Wo * stablePlant.A - Wo + CtC;

    float maxResidual{ 0.0f };
    for (std::size_t r = 0; r < 2; ++r)
        for (std::size_t c = 0; c < 2; ++c)
        {
            float v = residual.at(r, c);
            if (v < 0.0f) v = -v;
            if (v > maxResidual) maxResidual = v;
        }

    EXPECT_NEAR(maxResidual, 0.0f, 1e-3f);
}

TEST_F(TestControllabilityObservability, gramian_is_symmetric_positive_definite)
{
    LTI stablePlant;
    stablePlant.A = math::Matrix<float, 2, 2>{
        { 0.0f, 1.0f },
        { -0.2f, -0.3f }
    };
    stablePlant.B = math::Matrix<float, 2, 1>{
        { 0.0f },
        { 1.0f }
    };
    stablePlant.C = math::Matrix<float, 1, 2>{
        { 1.0f, 0.0f }
    };
    stablePlant.D = math::Matrix<float, 1, 1>{
        { 0.0f }
    };

    auto Wc = CO::ControllabilityGramian(stablePlant);

    EXPECT_NEAR(Wc.at(0, 1), Wc.at(1, 0), 1e-5f);
    EXPECT_TRUE(Wc.at(0, 0) > 0.0f);
    EXPECT_TRUE(Wc.at(1, 1) > 0.0f);
}
