#pragma once

#include "numerical/math/Matrix.hpp"
#include "numerical/optimization/BayesianOptimization.hpp"
#include "numerical/optimization/BlackBoxObjective.hpp"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace simulator::controllers
{
    struct MpcBoCalibratorConfig
    {
        float qMin = 0.1f;
        float qMax = 100.0f;
        float rMin = 0.01f;
        float rMax = 10.0f;
        std::size_t boIterations = 25;
        std::size_t closedLoopSteps = 50;
        uint64_t seed = 7777ULL;
    };

    struct MpcBoResult
    {
        float optimalQ;
        float optimalR;
        float finalIse;
        std::vector<float> iseHistory;
    };

    class MpcBoCalibrator
    {
    public:
        static constexpr std::size_t BoNumParams = 2;
        static constexpr std::size_t BoMaxObs = 30;
        static constexpr std::size_t BoNumCandidates = 200;

        using Bo = optimization::BayesianOptimization<BoNumParams, BoMaxObs, BoNumCandidates>;

        MpcBoResult Calibrate(
            const math::SquareMatrix<float, 2>& F_est,
            const math::Matrix<float, 2, 1>& B_true,
            const math::SquareMatrix<float, 2>& A_true,
            const MpcBoCalibratorConfig& config);

    private:
        class IseObjective : public optimization::BlackBoxObjective<BoNumParams>
        {
        public:
            IseObjective(
                const math::SquareMatrix<float, 2>& F,
                const math::Matrix<float, 2, 1>& B,
                const math::SquareMatrix<float, 2>& A_true,
                std::size_t closedLoopSteps);

            float Evaluate(const math::Vector<float, BoNumParams>& params) override;

        private:
            float RunClosedLoopIse(float q, float r) const;

            math::SquareMatrix<float, 2> F_;
            math::Matrix<float, 2, 1> B_;
            math::SquareMatrix<float, 2> A_true_;
            std::size_t closedLoopSteps_;
        };
    };
}
