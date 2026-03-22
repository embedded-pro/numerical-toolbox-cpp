#pragma once

#include <cmath>
#include <vector>

namespace simulator::filters
{
    struct PendulumParameters
    {
        float length = 1.0f;
        float mass = 1.0f;
        float damping = 0.1f;
        float gravity = 9.81f;
    };

    struct PendulumState
    {
        float theta = 0.0f;
        float thetaDot = 0.0f;
    };

    class PendulumPlant
    {
    public:
        explicit PendulumPlant(PendulumParameters params = PendulumParameters{});

        PendulumState Step(float torque, float dt);
        void Reset(float theta0, float thetaDot0);
        PendulumState GetState() const;

    private:
        PendulumParameters parameters;
        PendulumState state;
    };
}
