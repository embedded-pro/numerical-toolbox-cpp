#include "simulator/controllers/PidController/application/PidSimulator.hpp"
#include "numerical/analysis/FrequencyResponse.hpp"
#include "numerical/analysis/RootLocus.hpp"
#include "numerical/controllers/implementations/PidIncremental.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace simulator::controllers
{
    namespace
    {
        std::vector<float> PolyMultiply(const std::vector<float>& a, const std::vector<float>& b)
        {
            if (a.empty() || b.empty())
                return {};

            std::vector<float> result(a.size() + b.size() - 1, 0.0f);
            for (std::size_t i = 0; i < a.size(); ++i)
                for (std::size_t j = 0; j < b.size(); ++j)
                    result[i + j] += a[i] * b[j];

            return result;
        }
    }

    void PidSimulator::Configure(std::unique_ptr<Plant> plant, const Configuration& config)
    {
        this->plant = std::move(plant);
        configuration = config;
    }

    TransferFunction PidSimulator::GetPidTf() const
    {
        float kp = configuration.tunings.kp;
        float ki = configuration.tunings.ki;
        float kd = configuration.tunings.kd;

        return { { kd, kp, ki }, { 1.0f, 0.0f } };
    }

    TransferFunction PidSimulator::GetOpenLoopTf() const
    {
        auto plantTf = plant->GetTransferFunction();
        auto pidTf = GetPidTf();

        return {
            PolyMultiply(pidTf.numerator, plantTf.numerator),
            PolyMultiply(pidTf.denominator, plantTf.denominator)
        };
    }

    TransferFunction PidSimulator::DiscretizeOpenLoop() const
    {
        auto openLoop = GetOpenLoopTf();
        float T = configuration.simulation.sampleTime;
        float c = 2.0f / T;

        std::size_t numOrder = openLoop.numerator.size();
        std::size_t denOrder = openLoop.denominator.size();
        std::size_t maxOrder = std::max(numOrder, denOrder);

        std::vector<float> num(maxOrder, 0.0f);
        std::vector<float> den(maxOrder, 0.0f);
        for (std::size_t i = 0; i < numOrder; ++i)
            num[maxOrder - numOrder + i] = openLoop.numerator[i];
        for (std::size_t i = 0; i < denOrder; ++i)
            den[maxOrder - denOrder + i] = openLoop.denominator[i];

        std::size_t N = maxOrder;
        std::vector<float> numZ(N, 0.0f);
        std::vector<float> denZ(N, 0.0f);

        for (std::size_t k = 0; k < N; ++k)
        {
            std::size_t power = N - 1 - k;

            std::vector<float> term = { 1.0f };
            for (std::size_t p = 0; p < power; ++p)
                term = PolyMultiply(term, { 1.0f, -1.0f });
            for (std::size_t p = 0; p < (N - 1 - power); ++p)
                term = PolyMultiply(term, { 1.0f, 1.0f });

            float cPow = std::pow(c, static_cast<float>(power));

            for (std::size_t i = 0; i < term.size() && i < N; ++i)
            {
                numZ[i] += num[k] * cPow * term[i];
                denZ[i] += den[k] * cPow * term[i];
            }
        }

        float norm = denZ[0];
        if (std::abs(norm) > 1e-30f)
        {
            for (auto& v : numZ)
                v /= norm;
            for (auto& v : denZ)
                v /= norm;
        }

        return { numZ, denZ };
    }

    TimeResponse PidSimulator::SimulateClosedLoop(const std::vector<float>& referenceSignal)
    {
        float dt = configuration.simulation.sampleTime;
        std::size_t numSamples = referenceSignal.size();

        TimeResponse response;
        response.time.resize(numSamples);
        response.reference = referenceSignal;
        response.output.resize(numSamples, 0.0f);
        response.controlSignal.resize(numSamples, 0.0f);
        response.error.resize(numSamples, 0.0f);

        ::controllers::PidIncrementalSynchronous<float> pid(configuration.tunings, configuration.limits);
        pid.Enable();

        plant->Reset();

        for (std::size_t i = 0; i < numSamples; ++i)
        {
            response.time[i] = static_cast<float>(i) * dt;

            float y = plant->Output();
            response.output[i] = y;
            response.error[i] = referenceSignal[i] - y;

            pid.SetPoint(referenceSignal[i]);
            float u = pid.Process(y);
            response.controlSignal[i] = u;

            plant->Step(u, dt);
        }

        return response;
    }

    TimeResponse PidSimulator::ComputeStepResponse()
    {
        float dt = configuration.simulation.sampleTime;
        float duration = configuration.simulation.duration;
        std::size_t numSamples = static_cast<std::size_t>(duration / dt);

        std::vector<float> reference(numSamples, 1.0f);
        return SimulateClosedLoop(reference);
    }

    TimeResponse PidSimulator::ComputeRampResponse()
    {
        float dt = configuration.simulation.sampleTime;
        float duration = configuration.simulation.duration;
        std::size_t numSamples = static_cast<std::size_t>(duration / dt);

        std::vector<float> reference(numSamples);
        for (std::size_t i = 0; i < numSamples; ++i)
            reference[i] = static_cast<float>(i) * dt;

        return SimulateClosedLoop(reference);
    }

    BodeResult PidSimulator::ComputeBodeResponse()
    {
        auto discrete = DiscretizeOpenLoop();
        float fs = 1.0f / configuration.simulation.sampleTime;

        std::vector<float> numCoeffs = discrete.numerator;
        std::vector<float> denCoeffs = discrete.denominator;

        analysis::FrequencyResponse<float, bodeResolution> freqResponse(
            infra::MemoryRange<float>(numCoeffs),
            infra::MemoryRange<float>(denCoeffs),
            fs);

        auto [frequencies, magnitudeDb, phaseDeg] = freqResponse.Calculate();

        BodeResult result;
        result.frequencies.assign(frequencies.begin(), frequencies.end());
        result.magnitudeDb.assign(magnitudeDb.begin(), magnitudeDb.end());
        result.phaseDeg.assign(phaseDeg.begin(), phaseDeg.end());

        float prevMagDb = 0.0f;
        float prevPhaseDeg = 0.0f;

        for (std::size_t i = 0; i < result.frequencies.size(); ++i)
        {
            float magDb = result.magnitudeDb[i];
            float phase = result.phaseDeg[i];

            if (i > 0 && prevMagDb > 0.0f && magDb <= 0.0f)
            {
                float t = prevMagDb / (prevMagDb - magDb);
                result.phaseMarginDeg = 180.0f + (prevPhaseDeg + t * (phase - prevPhaseDeg));
                result.gainCrossoverHz = result.frequencies[i];
            }

            if (i > 0 && prevPhaseDeg > -180.0f && phase <= -180.0f)
            {
                float t = (prevPhaseDeg + 180.0f) / (prevPhaseDeg - phase);
                float magAtCrossover = prevMagDb + t * (magDb - prevMagDb);
                result.gainMarginDb = -magAtCrossover;
                result.phaseCrossoverHz = result.frequencies[i];
            }

            prevMagDb = magDb;
            prevPhaseDeg = phase;
        }

        return result;
    }

    RootLocusResult PidSimulator::ComputeRootLocus()
    {
        auto pidTf = GetPidTf();
        auto plantTf = plant->GetTransferFunction();

        auto openNum = PolyMultiply(pidTf.numerator, plantTf.numerator);
        auto openDen = PolyMultiply(pidTf.denominator, plantTf.denominator);

        analysis::RootLocus<float, rootLocusMaxOrder, rootLocusGainSteps> rootLocus;
        auto rlResult = rootLocus.Calculate(
            infra::MemoryRange<const float>(openNum),
            infra::MemoryRange<const float>(openDen),
            1.0f);

        RootLocusResult result;
        result.currentGain = rlResult.currentGain;
        result.gains.assign(rlResult.gains.begin(), rlResult.gains.end());
        result.openLoopPoles.assign(rlResult.openLoopPoles.begin(), rlResult.openLoopPoles.end());
        result.openLoopZeros.assign(rlResult.openLoopZeros.begin(), rlResult.openLoopZeros.end());
        result.closedLoopPoles.assign(rlResult.closedLoopPoles.begin(), rlResult.closedLoopPoles.end());

        result.loci.resize(rlResult.activeBranches);
        for (std::size_t branch = 0; branch < rlResult.activeBranches; ++branch)
            result.loci[branch].assign(rlResult.loci[branch].begin(), rlResult.loci[branch].end());

        return result;
    }
}
