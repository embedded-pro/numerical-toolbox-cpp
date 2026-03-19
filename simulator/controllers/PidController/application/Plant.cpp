#include "simulator/controllers/PidController/application/Plant.hpp"
#include <cmath>

namespace simulator::controllers
{
    FirstOrderPlant::FirstOrderPlant(float gain, float timeConstant)
        : gain(gain)
        , timeConstant(timeConstant)
    {
    }

    TransferFunction FirstOrderPlant::GetTransferFunction() const
    {
        return { { gain }, { timeConstant, 1.0f } };
    }

    float FirstOrderPlant::Output() const
    {
        return state;
    }

    void FirstOrderPlant::Step(float input, float dt)
    {
        float dx = (-state + gain * input) / timeConstant;
        state += dx * dt;
    }

    void FirstOrderPlant::Reset()
    {
        state = 0.0f;
    }

    std::vector<std::complex<float>> FirstOrderPlant::Poles() const
    {
        return { { -1.0f / timeConstant, 0.0f } };
    }

    SecondOrderPlant::SecondOrderPlant(float naturalFrequency, float dampingRatio)
        : naturalFrequency(naturalFrequency)
        , dampingRatio(dampingRatio)
    {
    }

    TransferFunction SecondOrderPlant::GetTransferFunction() const
    {
        float wn2 = naturalFrequency * naturalFrequency;
        return { { wn2 }, { 1.0f, 2.0f * dampingRatio * naturalFrequency, wn2 } };
    }

    float SecondOrderPlant::Output() const
    {
        return x1;
    }

    void SecondOrderPlant::Step(float input, float dt)
    {
        float wn2 = naturalFrequency * naturalFrequency;
        float dx1 = x2;
        float dx2 = -2.0f * dampingRatio * naturalFrequency * x2 - wn2 * x1 + wn2 * input;
        x1 += dx1 * dt;
        x2 += dx2 * dt;
    }

    void SecondOrderPlant::Reset()
    {
        x1 = 0.0f;
        x2 = 0.0f;
    }

    std::vector<std::complex<float>> SecondOrderPlant::Poles() const
    {
        float wn = naturalFrequency;
        float zeta = dampingRatio;
        float disc = zeta * zeta - 1.0f;

        if (disc >= 0)
            return {
                { -zeta * wn + wn * std::sqrt(disc), 0.0f },
                { -zeta * wn - wn * std::sqrt(disc), 0.0f }
            };

        float realPart = -zeta * wn;
        float imagPart = wn * std::sqrt(-disc);
        return {
            { realPart, imagPart },
            { realPart, -imagPart }
        };
    }
}
