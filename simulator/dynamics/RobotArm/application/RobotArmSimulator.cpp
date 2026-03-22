#include "simulator/dynamics/RobotArm/application/RobotArmSimulator.hpp"

namespace simulator::dynamics
{
    void RobotArmSimulator::Configure(const RobotArmConfig& cfg)
    {
        config = cfg;
        torques.assign(config.dof, 0.0f);
        Reset();
    }

    void RobotArmSimulator::SetTorques(const std::vector<float>& t)
    {
        for (int i = 0; i < config.dof && i < static_cast<int>(t.size()); ++i)
            torques[i] = t[i];
    }

    void RobotArmSimulator::SetInitialPositions(const std::vector<float>& positions)
    {
        for (int i = 0; i < config.dof && i < static_cast<int>(positions.size()); ++i)
            state.q[i] = positions[i];

        state.qDot.assign(config.dof, 0.0f);
        UpdateForwardKinematics();
    }

    void RobotArmSimulator::Step()
    {
        if (config.dof == 2)
            StepDof2();
        else
            StepDof3();

        for (int i = 0; i < config.dof; ++i)
            state.qDot[i] *= (1.0f - config.damping * config.dt);

        state.time += config.dt;
        UpdateForwardKinematics();
        UpdateTrail();
    }

    void RobotArmSimulator::Reset()
    {
        state.q.assign(config.dof, 0.0f);
        state.qDot.assign(config.dof, 0.0f);
        state.qDDot.assign(config.dof, 0.0f);
        state.inverseDynamicsTorques.assign(config.dof, 0.0f);
        state.time = 0.0f;
        state.endEffectorTrail.clear();
        UpdateForwardKinematics();
    }

    const RobotArmState& RobotArmSimulator::GetState() const
    {
        return state;
    }

    const RobotArmConfig& RobotArmSimulator::GetConfig() const
    {
        return config;
    }

    void RobotArmSimulator::StepDof2()
    {
        auto links = BuildLinks2();

        math::Vector<float, 2> q{ { state.q[0], state.q[1] } };
        math::Vector<float, 2> qDot{ { state.qDot[0], state.qDot[1] } };
        math::Vector<float, 2> tau{ { torques[0], torques[1] } };
        math::Vector<float, 3> g{ { 0.0f, 0.0f, -gravity } };

        // ABA: forward dynamics — compute accelerations from applied torques
        auto qDDot = aba2.ForwardDynamics(links, q, qDot, tau, g);

        // Semi-implicit Euler integration
        for (int i = 0; i < 2; ++i)
        {
            state.qDDot[i] = qDDot.at(i, 0);
            state.qDot[i] += qDDot.at(i, 0) * config.dt;
            state.q[i] += state.qDot[i] * config.dt;
        }

        // RNEA: inverse dynamics — compute torques required for current motion
        math::Vector<float, 2> qNew{ { state.q[0], state.q[1] } };
        math::Vector<float, 2> qDotNew{ { state.qDot[0], state.qDot[1] } };
        auto idTorques = rnea2.InverseDynamics(links, qNew, qDotNew, qDDot, g);

        for (int i = 0; i < 2; ++i)
            state.inverseDynamicsTorques[i] = idTorques.at(i, 0);
    }

    void RobotArmSimulator::StepDof3()
    {
        auto links = BuildLinks3();

        math::Vector<float, 3> q{ { state.q[0], state.q[1], state.q[2] } };
        math::Vector<float, 3> qDot{ { state.qDot[0], state.qDot[1], state.qDot[2] } };
        math::Vector<float, 3> tau{ { torques[0], torques[1], torques[2] } };
        math::Vector<float, 3> g{ { 0.0f, 0.0f, -gravity } };

        // ABA: forward dynamics — compute accelerations from applied torques
        auto qDDot = aba3.ForwardDynamics(links, q, qDot, tau, g);

        for (int i = 0; i < 3; ++i)
        {
            state.qDDot[i] = qDDot.at(i, 0);
            state.qDot[i] += qDDot.at(i, 0) * config.dt;
            state.q[i] += state.qDot[i] * config.dt;
        }

        // RNEA: inverse dynamics — compute torques required for current motion
        math::Vector<float, 3> qNew{ { state.q[0], state.q[1], state.q[2] } };
        math::Vector<float, 3> qDotNew{ { state.qDot[0], state.qDot[1], state.qDot[2] } };
        auto idTorques = rnea3.InverseDynamics(links, qNew, qDotNew, qDDot, g);

        for (int i = 0; i < 3; ++i)
            state.inverseDynamicsTorques[i] = idTorques.at(i, 0);
    }

    std::array<::dynamics::RevoluteJointLink<float>, 2> RobotArmSimulator::BuildLinks2() const
    {
        std::array<::dynamics::RevoluteJointLink<float>, 2> links;

        float L0 = config.linkLengths[0];
        float m0 = config.linkMasses[0];
        float I0 = m0 * L0 * L0 / 12.0f;

        links[0].mass = m0;
        links[0].inertia = math::SquareMatrix<float, 3>{ { { I0, 0, 0 }, { 0, 0.001f, 0 }, { 0, 0, I0 } } };
        links[0].jointAxis = math::Vector<float, 3>{ { 0.0f, 1.0f, 0.0f } };
        links[0].parentToJoint = math::Vector<float, 3>{ { 0.0f, 0.0f, 0.0f } };
        links[0].jointToCoM = math::Vector<float, 3>{ { L0 / 2.0f, 0.0f, 0.0f } };

        float L1 = config.linkLengths[1];
        float m1 = config.linkMasses[1];
        float I1 = m1 * L1 * L1 / 12.0f;

        links[1].mass = m1;
        links[1].inertia = math::SquareMatrix<float, 3>{ { { I1, 0, 0 }, { 0, 0.001f, 0 }, { 0, 0, I1 } } };
        links[1].jointAxis = math::Vector<float, 3>{ { 0.0f, 1.0f, 0.0f } };
        links[1].parentToJoint = math::Vector<float, 3>{ { L0, 0.0f, 0.0f } };
        links[1].jointToCoM = math::Vector<float, 3>{ { L1 / 2.0f, 0.0f, 0.0f } };

        return links;
    }

    std::array<::dynamics::RevoluteJointLink<float>, 3> RobotArmSimulator::BuildLinks3() const
    {
        std::array<::dynamics::RevoluteJointLink<float>, 3> links;

        float L0 = config.linkLengths[0];
        float m0 = config.linkMasses[0];
        float I0 = m0 * L0 * L0 / 12.0f;

        links[0].mass = m0;
        links[0].inertia = math::SquareMatrix<float, 3>{ { { I0, 0, 0 }, { 0, I0, 0 }, { 0, 0, 0.001f } } };
        links[0].jointAxis = math::Vector<float, 3>{ { 0.0f, 0.0f, 1.0f } };
        links[0].parentToJoint = math::Vector<float, 3>{ { 0.0f, 0.0f, 0.0f } };
        links[0].jointToCoM = math::Vector<float, 3>{ { 0.0f, 0.0f, L0 / 2.0f } };

        float L1 = config.linkLengths[1];
        float m1 = config.linkMasses[1];
        float I1 = m1 * L1 * L1 / 12.0f;

        links[1].mass = m1;
        links[1].inertia = math::SquareMatrix<float, 3>{ { { I1, 0, 0 }, { 0, 0.001f, 0 }, { 0, 0, I1 } } };
        links[1].jointAxis = math::Vector<float, 3>{ { 0.0f, 1.0f, 0.0f } };
        links[1].parentToJoint = math::Vector<float, 3>{ { 0.0f, 0.0f, L0 } };
        links[1].jointToCoM = math::Vector<float, 3>{ { L1 / 2.0f, 0.0f, 0.0f } };

        float L2 = config.linkLengths[2];
        float m2 = config.linkMasses[2];
        float I2 = m2 * L2 * L2 / 12.0f;

        links[2].mass = m2;
        links[2].inertia = math::SquareMatrix<float, 3>{ { { I2, 0, 0 }, { 0, 0.001f, 0 }, { 0, 0, I2 } } };
        links[2].jointAxis = math::Vector<float, 3>{ { 0.0f, 1.0f, 0.0f } };
        links[2].parentToJoint = math::Vector<float, 3>{ { L1, 0.0f, 0.0f } };
        links[2].jointToCoM = math::Vector<float, 3>{ { L2 / 2.0f, 0.0f, 0.0f } };

        return links;
    }

    void RobotArmSimulator::UpdateForwardKinematics()
    {
        int n = config.dof;
        state.jointPositions.resize(n + 1);

        if (n == 2)
        {
            auto links = BuildLinks2();
            math::Vector<float, 2> q{ { state.q[0], state.q[1] } };
            auto positions = fk2.Compute(links, q);

            for (int i = 0; i <= n; ++i)
                state.jointPositions[i] = { positions[i].at(0, 0), positions[i].at(1, 0), positions[i].at(2, 0) };
        }
        else
        {
            auto links = BuildLinks3();
            math::Vector<float, 3> q{ { state.q[0], state.q[1], state.q[2] } };
            auto positions = fk3.Compute(links, q);

            for (int i = 0; i <= n; ++i)
                state.jointPositions[i] = { positions[i].at(0, 0), positions[i].at(1, 0), positions[i].at(2, 0) };
        }
    }

    void RobotArmSimulator::UpdateTrail()
    {
        if (!state.jointPositions.empty())
        {
            state.endEffectorTrail.push_back(state.jointPositions.back());
            if (state.endEffectorTrail.size() > maxTrailSize)
                state.endEffectorTrail.erase(state.endEffectorTrail.begin());
        }
    }
}
