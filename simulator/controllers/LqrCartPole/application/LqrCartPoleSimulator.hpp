#pragma once

#include "numerical/controllers/implementations/Lqr.hpp"
#include "simulator/controllers/LqrCartPole/application/CartPolePlant.hpp"
#include <memory>

namespace simulator::controllers::lqr
{
    using LqrController = ::controllers::Lqr<float, stateSize, inputSize>;

    struct LqrWeightsConfig
    {
        float qX = 1.0f;
        float qXDot = 1.0f;
        float qTheta = 100.0f;
        float qThetaDot = 10.0f;
        float rForce = 0.01f;
    };

    struct LqrSimulationConfig
    {
        float dt = 0.01f;
        float forceLimit = 50.0f;
    };

    struct LqrCartPoleConfig
    {
        CartPoleParameters plantParams;
        LqrWeightsConfig weights;
        LqrSimulationConfig simulation;
    };

    class LqrCartPoleSimulator
    {
    public:
        LqrCartPoleSimulator();

        void Configure(const LqrCartPoleConfig& config);
        void Reset();
        void Step(float externalForce = 0.0f);
        void SetState(const CartPoleState& state);

        [[nodiscard]] float ComputeControlForce() const;
        [[nodiscard]] const CartPoleState& GetState() const;
        [[nodiscard]] const CartPoleParameters& GetPlantParameters() const;
        [[nodiscard]] float GetDt() const;
        [[nodiscard]] float GetForceLimit() const;

    private:
        void RecomputeGain();

        LqrCartPoleConfig configuration;
        CartPolePlant plant;
        std::unique_ptr<LqrController> lqr;
    };
}
