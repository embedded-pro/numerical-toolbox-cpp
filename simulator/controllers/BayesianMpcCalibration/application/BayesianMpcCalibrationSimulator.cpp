#include "simulator/controllers/BayesianMpcCalibration/application/BayesianMpcCalibrationSimulator.hpp"
#include "numerical/controllers/implementations/Mpc.hpp"
#include <algorithm>

namespace simulator::controllers
{
    CalibrationSimulationResults BayesianMpcCalibrationSimulator::Run(
        const CalibrationSimulationConfig& config)
    {
        CalibrationSimulationResults results{};

        DoubleIntegratorPlant plant(config.plant);
        const auto A_true = plant.GetA();
        const auto B_true = plant.GetB();

        std::array<MeasurementVector, MaxSteps> observations{};
        plant.GenerateObservations(
            config.numTrainingSteps, config.plantSeed,
            observations, results.truePositions);

        const std::size_t effectiveSteps = std::min(config.numTrainingSteps, MaxSteps);
        results.timeAxis.reserve(effectiveSteps);
        results.rawMeasurements.reserve(effectiveSteps);
        for (std::size_t t = 0; t < effectiveSteps; ++t)
        {
            results.timeAxis.push_back(static_cast<float>(t) * config.plant.dt);
            results.rawMeasurements.push_back(observations[t].at(0, 0));
        }

        const auto initialGuess = BuildInitialEmGuess(config.plant);

        EmKfPipeline emPipeline;
        const auto emKfResult = emPipeline.Run(
            observations, effectiveSteps, initialGuess, config.emKf);

        results.emLogLikelihoodHistory = emKfResult.logLikelihoodHistory;
        results.emConverged = emKfResult.emResult.converged;
        results.emIterations = emKfResult.emResult.iterations;

        const auto F_est = emKfResult.emResult.parameters.F;

        MpcBoCalibrator boCalibrator;
        const auto boResult = boCalibrator.Calibrate(F_est, B_true, A_true, config.mpcBo);

        results.boIseHistory = boResult.iseHistory;
        results.optimalQ = boResult.optimalQ;
        results.optimalR = boResult.optimalR;
        results.finalIse = boResult.finalIse;

        RunFinalStepResponse(A_true, B_true, F_est,
            boResult.optimalQ, boResult.optimalR, config.plant.dt, results);

        return results;
    }

    EmKfPipeline::EmParameters BayesianMpcCalibrationSimulator::BuildInitialEmGuess(
        const DoubleIntegratorConfig& plantConfig) const
    {
        using StateMatrix = math::SquareMatrix<float, 2>;
        using MeasurementMatrix = math::Matrix<float, 1, 2>;
        using StateVector = math::Vector<float, 2>;
        using MeasCovariance = math::SquareMatrix<float, 1>;

        EmKfPipeline::EmParameters guess{};
        guess.F = StateMatrix::Identity();
        guess.H = MeasurementMatrix{ { 1.0f, 0.0f } };
        guess.Q = StateMatrix::Identity() * (plantConfig.sigmaQ * plantConfig.sigmaQ);
        guess.R = MeasCovariance{ { plantConfig.sigmaR * plantConfig.sigmaR } };
        guess.initialState = StateVector{};
        guess.initialCovariance = StateMatrix::Identity();
        return guess;
    }

    void BayesianMpcCalibrationSimulator::RunFinalStepResponse(
        const math::SquareMatrix<float, 2>& A_true,
        const math::Matrix<float, 2, 1>& B_true,
        const math::SquareMatrix<float, 2>& F_est,
        float optimalQ,
        float optimalR,
        float dt,
        CalibrationSimulationResults& results) const
    {
        using WeightsType = ::controllers::MpcWeights<float, 2, 1>;
        WeightsType weights;
        weights.Q = math::SquareMatrix<float, 2>::Identity() * optimalQ;
        weights.R = math::SquareMatrix<float, 1>{ { optimalR } };

        ::controllers::Mpc<float, 2, 1, 10, 5> mpc(F_est, B_true, weights);
        const math::Vector<float, 2> ref{ { 1.0f }, { 0.0f } };
        mpc.SetReference(ref);

        math::Vector<float, 2> state{};
        results.stepResponseTime.reserve(StepResponseSteps);
        results.stepResponsePosition.reserve(StepResponseSteps);
        results.stepResponseVelocity.reserve(StepResponseSteps);
        results.stepResponseControl.reserve(StepResponseSteps);

        for (std::size_t k = 0; k < StepResponseSteps; ++k)
        {
            results.stepResponseTime.push_back(static_cast<float>(k) * dt);
            results.stepResponsePosition.push_back(state.at(0, 0));
            results.stepResponseVelocity.push_back(state.at(1, 0));

            const auto u = mpc.ComputeControl(state);
            results.stepResponseControl.push_back(u.at(0, 0));

            state = A_true * state + B_true * u;
        }
    }
}
