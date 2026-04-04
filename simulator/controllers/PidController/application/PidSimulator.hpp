#pragma once

#include "numerical/controllers/interfaces/PidController.hpp"
#include "simulator/controllers/common/Plant.hpp"
#include <complex>
#include <cstddef>
#include <memory>
#include <vector>

namespace simulator::controllers
{
    struct SimulationConfig
    {
        float duration = 20.0f;
        float sampleTime = 0.01f;
    };

    struct TimeResponse
    {
        std::vector<float> time;
        std::vector<float> reference;
        std::vector<float> output;
        std::vector<float> controlSignal;
        std::vector<float> error;
    };

    struct BodeResult
    {
        std::vector<float> frequencies;
        std::vector<float> magnitudeDb;
        std::vector<float> phaseDeg;
        float gainMarginDb = 0.0f;
        float phaseMarginDeg = 0.0f;
        float gainCrossoverHz = 0.0f;
        float phaseCrossoverHz = 0.0f;
    };

    struct RootLocusResult
    {
        std::vector<std::vector<std::complex<float>>> loci;
        std::vector<float> gains;
        std::vector<std::complex<float>> openLoopPoles;
        std::vector<std::complex<float>> openLoopZeros;
        std::vector<std::complex<float>> closedLoopPoles;
        float currentGain = 0.0f;
    };

    class PidSimulator
    {
    public:
        static constexpr std::size_t bodeResolution = 500;
        static constexpr std::size_t rootLocusMaxOrder = 10;
        static constexpr std::size_t rootLocusGainSteps = 300;

        struct Configuration
        {
            ::controllers::PidTunings<float> tunings;
            ::controllers::PidLimits<float> limits;
            SimulationConfig simulation;
        };

        void Configure(std::unique_ptr<Plant> plant, const Configuration& config);

        TimeResponse ComputeStepResponse();
        TimeResponse ComputeRampResponse();
        BodeResult ComputeBodeResponse();
        RootLocusResult ComputeRootLocus();

    private:
        TransferFunction GetPidTf() const;
        TransferFunction GetOpenLoopTf() const;
        TransferFunction DiscretizeOpenLoop() const;
        TimeResponse SimulateClosedLoop(const std::vector<float>& referenceSignal);

        std::unique_ptr<Plant> plant;
        Configuration configuration;
    };
}
