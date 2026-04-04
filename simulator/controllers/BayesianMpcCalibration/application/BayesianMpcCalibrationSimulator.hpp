#pragma once

#include "simulator/controllers/BayesianMpcCalibration/application/DoubleIntegratorPlant.hpp"
#include "simulator/controllers/BayesianMpcCalibration/application/EmKfPipeline.hpp"
#include "simulator/controllers/BayesianMpcCalibration/application/MpcBoCalibrator.hpp"
#include <vector>

namespace simulator::controllers
{
    struct CalibrationSimulationConfig
    {
        DoubleIntegratorConfig plant;
        EmKfConfig emKf;
        MpcBoCalibratorConfig mpcBo;
        std::size_t numTrainingSteps = 150;
        uint32_t plantSeed = 42U;
    };

    struct CalibrationSimulationResults
    {
        std::vector<float> timeAxis;
        std::vector<float> rawMeasurements;
        std::vector<float> truePositions;

        std::vector<float> emLogLikelihoodHistory;
        bool emConverged = false;
        std::size_t emIterations = 0;

        std::vector<float> boIseHistory;
        float optimalQ = 1.0f;
        float optimalR = 1.0f;
        float finalIse = 0.0f;

        std::vector<float> stepResponseTime;
        std::vector<float> stepResponsePosition;
        std::vector<float> stepResponseVelocity;
        std::vector<float> stepResponseControl;
    };

    class BayesianMpcCalibrationSimulator
    {
    public:
        CalibrationSimulationResults Run(const CalibrationSimulationConfig& config);

    private:
        static constexpr std::size_t MaxSteps = DoubleIntegratorPlant::MaxSteps;
        static constexpr std::size_t StepResponseSteps = 100;

        using MeasurementVector = math::Vector<float, 1>;

        EmKfPipeline::EmParameters BuildInitialEmGuess(const DoubleIntegratorConfig& plantConfig) const;

        void RunFinalStepResponse(
            const math::SquareMatrix<float, 2>& A_true,
            const math::Matrix<float, 2, 1>& B_true,
            const math::SquareMatrix<float, 2>& F_est,
            float optimalQ,
            float optimalR,
            float dt,
            CalibrationSimulationResults& results) const;
    };
}
