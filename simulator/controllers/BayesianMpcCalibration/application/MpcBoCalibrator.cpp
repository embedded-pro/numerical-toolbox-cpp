#include "simulator/controllers/BayesianMpcCalibration/application/MpcBoCalibrator.hpp"
#include "numerical/controllers/implementations/Mpc.hpp"
#include <algorithm>
#include <cmath>

namespace simulator::controllers
{
    MpcBoCalibrator::IseObjective::IseObjective(
        const math::SquareMatrix<float, 2>& F,
        const math::Matrix<float, 2, 1>& B,
        const math::SquareMatrix<float, 2>& A_true,
        std::size_t closedLoopSteps)
        : F_(F)
        , B_(B)
        , A_true_(A_true)
        , closedLoopSteps_(closedLoopSteps)
    {}

    float MpcBoCalibrator::IseObjective::Evaluate(const math::Vector<float, BoNumParams>& params)
    {
        const float q = std::max(params.at(0, 0), 0.01f);
        const float r = std::max(params.at(1, 0), 0.001f);
        return RunClosedLoopIse(q, r);
    }

    float MpcBoCalibrator::IseObjective::RunClosedLoopIse(float q, float r) const
    {
        using WeightsType = ::controllers::MpcWeights<float, 2, 1>;
        WeightsType weights;
        weights.Q = math::SquareMatrix<float, 2>::Identity() * q;
        weights.R = math::SquareMatrix<float, 1>{ { r } };

        ::controllers::Mpc<float, 2, 1, 10, 5> mpc(F_, B_, weights);

        const math::Vector<float, 2> ref{ { 1.0f }, { 0.0f } };
        mpc.SetReference(ref);

        math::Vector<float, 2> state{ { 0.0f }, { 0.0f } };
        float ise = 0.0f;

        for (std::size_t k = 0; k < closedLoopSteps_; ++k)
        {
            const auto u = mpc.ComputeControl(state);
            state = A_true_ * state + B_ * u;

            const float ep = state.at(0, 0) - ref.at(0, 0);
            const float ev = state.at(1, 0) - ref.at(1, 0);
            ise += ep * ep + ev * ev;
        }

        return ise;
    }

    MpcBoResult MpcBoCalibrator::Calibrate(
        const math::SquareMatrix<float, 2>& F_est,
        const math::Matrix<float, 2, 1>& B_true,
        const math::SquareMatrix<float, 2>& A_true,
        const MpcBoCalibratorConfig& config)
    {
        IseObjective objective(F_est, B_true, A_true, config.closedLoopSteps);

        optimization::GpHyperparameters hp{ 2.0f, 1.0f, 0.05f };
        Bo::BoundsArray bounds{
            std::make_pair(config.qMin, config.qMax),
            std::make_pair(config.rMin, config.rMax)
        };
        Bo bo(bounds, hp, config.seed);

        really_assert(config.boIterations > 0 && config.boIterations <= BoMaxObs);
        const auto boResult = bo.Optimize(objective, config.boIterations);

        MpcBoResult result{};
        result.optimalQ = boResult.bestPoint.at(0, 0);
        result.optimalR = boResult.bestPoint.at(1, 0);
        result.finalIse = boResult.bestValue;

        result.iseHistory.reserve(bo.GetNumObservations());
        for (std::size_t i = 0; i < bo.GetNumObservations(); ++i)
            result.iseHistory.push_back(bo.GetObservedValues()[i]);

        return result;
    }
}
