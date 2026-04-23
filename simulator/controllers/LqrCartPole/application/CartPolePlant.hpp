#pragma once

#include "numerical/math/LinearTimeInvariant.hpp"
#include "numerical/math/Matrix.hpp"

namespace simulator::controllers::lqr
{
    static constexpr std::size_t stateSize = 4;
    static constexpr std::size_t inputSize = 1;

    using StateMatrix = math::SquareMatrix<float, stateSize>;
    using InputMatrix = math::Matrix<float, stateSize, inputSize>;
    using StateVector = math::Vector<float, stateSize>;

    struct CartPoleParameters
    {
        float cartMass = 1.0f;
        float poleMass = 0.1f;
        float poleLength = 0.5f;
        float gravity = 9.81f;
        float cartFriction = 0.1f;
        float trackLimit = 2.4f;
    };

    struct CartPoleState
    {
        float x = 0.0f;
        float xDot = 0.0f;
        float theta = 0.0f;
        float thetaDot = 0.0f;
    };

    class CartPolePlant
    {
    public:
        explicit CartPolePlant(const CartPoleParameters& params = {});

        void Step(float force, float dt);
        void Reset();
        void SetState(const CartPoleState& newState);

        [[nodiscard]] const CartPoleState& GetState() const;
        [[nodiscard]] const CartPoleParameters& GetParameters() const;

        [[nodiscard]] StateMatrix LinearizedA(float dt) const;
        [[nodiscard]] InputMatrix LinearizedB(float dt) const;
        [[nodiscard]] math::LinearTimeInvariant<float, stateSize, inputSize> Linearize(float dt) const;

    private:
        CartPoleParameters parameters;
        CartPoleState state;
    };
}
