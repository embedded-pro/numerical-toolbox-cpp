#include "simulator/controllers/LqrCartPole/application/CartPolePlant.hpp"
#include <cmath>
#include <numbers>

namespace simulator::controllers::lqr
{
    static constexpr auto pi = std::numbers::pi_v<float>;
    static constexpr auto twoPi = 2.0f * std::numbers::pi_v<float>;

    CartPolePlant::CartPolePlant(const CartPoleParameters& params)
        : parameters(params)
    {
    }

    void CartPolePlant::Step(float force, float dt)
    {
        auto mc = parameters.cartMass;
        auto mp = parameters.poleMass;
        auto l = parameters.poleLength;
        auto g = parameters.gravity;
        auto mu = parameters.cartFriction;

        auto sinTheta = std::sin(state.theta);
        auto cosTheta = std::cos(state.theta);
        auto totalMass = mc + mp;

        auto thetaDotSq = state.thetaDot * state.thetaDot;
        auto denom = totalMass - mp * cosTheta * cosTheta;

        auto xDDot = (force + mp * l * thetaDotSq * sinTheta - mp * g * cosTheta * sinTheta - mu * state.xDot) / denom;

        auto thetaDDot = (-force * cosTheta - mp * l * thetaDotSq * cosTheta * sinTheta + totalMass * g * sinTheta + mu * state.xDot * cosTheta) / (l * denom);

        state.xDot += xDDot * dt;
        state.x += state.xDot * dt;
        state.thetaDot += thetaDDot * dt;
        state.theta += state.thetaDot * dt;

        // Wrap theta to [-pi, pi]
        while (state.theta > pi)
            state.theta -= twoPi;
        while (state.theta < -pi)
            state.theta += twoPi;
    }

    void CartPolePlant::Reset()
    {
        state = {};
    }

    void CartPolePlant::SetState(const CartPoleState& newState)
    {
        state = newState;
    }

    const CartPoleState& CartPolePlant::GetState() const
    {
        return state;
    }

    const CartPoleParameters& CartPolePlant::GetParameters() const
    {
        return parameters;
    }

    StateMatrix CartPolePlant::LinearizedA(float dt) const
    {
        auto mc = parameters.cartMass;
        auto mp = parameters.poleMass;
        auto l = parameters.poleLength;
        auto g = parameters.gravity;
        auto mu = parameters.cartFriction;

        // Linearized around theta=0 (upright)
        // Continuous A matrix:
        // [0,  1,            0,  0]
        // [0, -mu/mc,   -mp*g/mc, 0]
        // [0,  0,            0,  1]
        // [0, mu/(mc*l), (mc+mp)*g/(mc*l), 0]

        auto a12 = -mu / mc;
        auto a13 = -mp * g / mc;
        auto a32 = mu / (mc * l);
        auto a33 = (mc + mp) * g / (mc * l);

        // Discrete A = I + Ac * dt (forward Euler)
        return StateMatrix{
            { 1.0f, dt, 0.0f, 0.0f },
            { 0.0f, 1.0f + a12 * dt, a13 * dt, 0.0f },
            { 0.0f, 0.0f, 1.0f, dt },
            { 0.0f, a32 * dt, a33 * dt, 1.0f },
        };
    }

    InputMatrix CartPolePlant::LinearizedB(float dt) const
    {
        auto mc = parameters.cartMass;
        auto l = parameters.poleLength;

        // Continuous B vector:
        // [0, 1/mc, 0, -1/(mc*l)]
        // Discrete B = Bc * dt
        return InputMatrix{
            { 0.0f },
            { dt / mc },
            { 0.0f },
            { -dt / (mc * l) },
        };
    }
}
