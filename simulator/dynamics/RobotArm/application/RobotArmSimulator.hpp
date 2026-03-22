#pragma once

#include "numerical/dynamics/ArticulatedBodyAlgorithm.hpp"
#include "numerical/dynamics/RecursiveNewtonEuler.hpp"
#include "numerical/dynamics/RevoluteJointLink.hpp"
#include "numerical/kinematics/ForwardKinematics.hpp"
#include <array>
#include <cstddef>
#include <vector>

namespace simulator::dynamics
{
    struct RobotArmConfig
    {
        int dof = 2;
        std::vector<float> linkLengths = { 0.5f, 0.4f, 0.3f };
        std::vector<float> linkMasses = { 1.0f, 0.8f, 0.6f };
        float damping = 0.05f;
        float dt = 1.0f / 120.0f;
    };

    struct RobotArmState
    {
        std::vector<std::array<float, 3>> jointPositions;
        std::vector<float> q;
        std::vector<float> qDot;
        std::vector<float> qDDot;
        std::vector<float> inverseDynamicsTorques;
        std::vector<std::array<float, 3>> endEffectorTrail;
        float time = 0.0f;
    };

    class RobotArmSimulator
    {
    public:
        void Configure(const RobotArmConfig& config);
        void SetTorques(const std::vector<float>& torques);
        void SetInitialPositions(const std::vector<float>& positions);
        void Step();
        void Reset();

        const RobotArmState& GetState() const;
        const RobotArmConfig& GetConfig() const;

    private:
        void StepDof2();
        void StepDof3();
        void UpdateForwardKinematics();
        void UpdateTrail();

        std::array<::dynamics::RevoluteJointLink<float>, 2> BuildLinks2() const;
        std::array<::dynamics::RevoluteJointLink<float>, 3> BuildLinks3() const;

        static constexpr float gravity = 9.81f;
        static constexpr std::size_t maxTrailSize = 500;

        ::dynamics::ArticulatedBodyAlgorithm<float, 2> aba2;
        ::dynamics::ArticulatedBodyAlgorithm<float, 3> aba3;
        ::dynamics::RecursiveNewtonEuler<float, 2> rnea2;
        ::dynamics::RecursiveNewtonEuler<float, 3> rnea3;
        ::kinematics::ForwardKinematics<float, 2> fk2;
        ::kinematics::ForwardKinematics<float, 3> fk3;

        RobotArmConfig config;
        RobotArmState state;
        std::vector<float> torques;
    };
}
