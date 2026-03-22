#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/dynamics/RevoluteJointLink.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Geometry3D.hpp"
#include "numerical/math/Matrix.hpp"
#include <array>

namespace dynamics
{
    template<typename T, std::size_t NumLinks>
    class RecursiveNewtonEuler
    {
        static_assert(std::is_floating_point_v<T>,
            "RecursiveNewtonEuler only supports floating-point types");
        static_assert(NumLinks > 0, "Number of links must be positive");

    public:
        using Vector3 = math::Vector<T, 3>;
        using Matrix3 = math::SquareMatrix<T, 3>;
        using JointVector = math::Vector<T, NumLinks>;
        using LinkArray = std::array<RevoluteJointLink<T>, NumLinks>;

        RecursiveNewtonEuler() = default;

        // Inverse dynamics: given joint positions, velocities, and accelerations,
        // compute the required joint torques. O(n) complexity.
        OPTIMIZE_FOR_SPEED JointVector InverseDynamics(const LinkArray& links,
            const JointVector& q, const JointVector& qDot, const JointVector& qDDot,
            const Vector3& gravity) const;

    private:
        struct LinkState
        {
            Vector3 omega;
            Vector3 omegaDot;
            Vector3 linAccCoM;
            Matrix3 R;
        };

        static void ComputeBaseLinkState(
            LinkState& state, const RevoluteJointLink<T>& link,
            T qDot_i, T qDDot_i, const Vector3& gravity);

        static void PropagateStateFromParent(
            LinkState& state, const LinkState& parent,
            const RevoluteJointLink<T>& link, const RevoluteJointLink<T>& parentLink,
            T qDot_i, T qDDot_i);

        static std::array<LinkState, NumLinks> ComputeForwardPass(
            const LinkArray& links, const JointVector& q,
            const JointVector& qDot, const JointVector& qDDot,
            const Vector3& gravity);

        static void ComputeLinkWrench(
            Vector3& force, Vector3& torque,
            const RevoluteJointLink<T>& link, const LinkState& state);

        static void AccumulateChildWrench(
            Vector3& parentForce, Vector3& parentTorque,
            const Vector3& childForce, const Vector3& childTorque,
            const Matrix3& childR, const Vector3& rToChild);

        static JointVector ComputeBackwardPass(
            const LinkArray& links, const std::array<LinkState, NumLinks>& states);
    };

    template<typename T, std::size_t NumLinks>
    void RecursiveNewtonEuler<T, NumLinks>::ComputeBaseLinkState(
        LinkState& state, const RevoluteJointLink<T>& link,
        T qDot_i, T qDDot_i, const Vector3& gravity)
    {
        auto Rt = state.R.Transpose();

        state.omega = link.jointAxis * qDot_i;

        auto qDotAxis = link.jointAxis * qDot_i;
        state.omegaDot = link.jointAxis * qDDot_i + math::CrossProduct(state.omega, qDotAxis);

        auto baseLinAcc = Rt * (gravity * T(-1));
        auto rCoM = link.jointToCoM;
        state.linAccCoM = baseLinAcc + math::CrossProduct(state.omegaDot, rCoM) + math::CrossProduct(state.omega, math::CrossProduct(state.omega, rCoM));
    }

    template<typename T, std::size_t NumLinks>
    void RecursiveNewtonEuler<T, NumLinks>::PropagateStateFromParent(
        LinkState& state, const LinkState& parent,
        const RevoluteJointLink<T>& link, const RevoluteJointLink<T>& parentLink,
        T qDot_i, T qDDot_i)
    {
        auto Rt = state.R.Transpose();
        auto qDotAxis = link.jointAxis * qDot_i;

        state.omega = Rt * parent.omega + qDotAxis;
        state.omegaDot = Rt * parent.omegaDot + math::CrossProduct(state.omega, qDotAxis) + link.jointAxis * qDDot_i;

        auto rParentCoM = parentLink.jointToCoM;
        auto parentJointAcc = parent.linAccCoM - math::CrossProduct(parent.omegaDot, rParentCoM) - math::CrossProduct(parent.omega, math::CrossProduct(parent.omega, rParentCoM));

        auto rToJoint = link.parentToJoint;
        auto accAtParentEnd = parentJointAcc + math::CrossProduct(parent.omegaDot, rToJoint) + math::CrossProduct(parent.omega, math::CrossProduct(parent.omega, rToJoint));

        auto accJointInLink = Rt * accAtParentEnd;
        auto rCoM = link.jointToCoM;
        state.linAccCoM = accJointInLink + math::CrossProduct(state.omegaDot, rCoM) + math::CrossProduct(state.omega, math::CrossProduct(state.omega, rCoM));
    }

    template<typename T, std::size_t NumLinks>
    std::array<typename RecursiveNewtonEuler<T, NumLinks>::LinkState, NumLinks>
    RecursiveNewtonEuler<T, NumLinks>::ComputeForwardPass(
        const LinkArray& links, const JointVector& q,
        const JointVector& qDot, const JointVector& qDDot,
        const Vector3& gravity)
    {
        std::array<LinkState, NumLinks> states{};

        for (std::size_t i = 0; i < NumLinks; ++i)
        {
            states[i].R = math::RotationAboutAxis(links[i].jointAxis, q.at(i, 0));

            if (i == 0)
                ComputeBaseLinkState(states[i], links[i], qDot.at(i, 0), qDDot.at(i, 0), gravity);
            else
                PropagateStateFromParent(states[i], states[i - 1], links[i], links[i - 1], qDot.at(i, 0), qDDot.at(i, 0));
        }

        return states;
    }

    template<typename T, std::size_t NumLinks>
    void RecursiveNewtonEuler<T, NumLinks>::ComputeLinkWrench(
        Vector3& force, Vector3& torque,
        const RevoluteJointLink<T>& link, const LinkState& state)
    {
        force = state.linAccCoM * link.mass;

        auto Iw = link.inertia * state.omega;
        torque = link.inertia * state.omegaDot + math::CrossProduct(state.omega, Iw);

        torque = torque + math::CrossProduct(link.jointToCoM, force);
    }

    template<typename T, std::size_t NumLinks>
    void RecursiveNewtonEuler<T, NumLinks>::AccumulateChildWrench(
        Vector3& parentForce, Vector3& parentTorque,
        const Vector3& childForce, const Vector3& childTorque,
        const Matrix3& childR, const Vector3& rToChild)
    {
        auto childForceInParent = childR * childForce;
        auto childTorqueInParent = childR * childTorque;

        parentForce = parentForce + childForceInParent;
        parentTorque = parentTorque + childTorqueInParent + math::CrossProduct(rToChild, childForceInParent);
    }

    template<typename T, std::size_t NumLinks>
    typename RecursiveNewtonEuler<T, NumLinks>::JointVector
    RecursiveNewtonEuler<T, NumLinks>::ComputeBackwardPass(
        const LinkArray& links, const std::array<LinkState, NumLinks>& states)
    {
        std::array<Vector3, NumLinks> force;
        std::array<Vector3, NumLinks> torque;

        for (std::size_t j = NumLinks; j > 0; --j)
        {
            std::size_t i = j - 1;

            ComputeLinkWrench(force[i], torque[i], links[i], states[i]);

            if (i + 1 < NumLinks)
                AccumulateChildWrench(force[i], torque[i], force[i + 1], torque[i + 1],
                    states[i + 1].R, links[i + 1].parentToJoint);
        }

        JointVector tau;
        for (std::size_t i = 0; i < NumLinks; ++i)
        {
            auto dotProduct = links[i].jointAxis.Transpose() * torque[i];
            tau.at(i, 0) = dotProduct.at(0, 0);
        }

        return tau;
    }

    template<typename T, std::size_t NumLinks>
    OPTIMIZE_FOR_SPEED
        typename RecursiveNewtonEuler<T, NumLinks>::JointVector
        RecursiveNewtonEuler<T, NumLinks>::InverseDynamics(const LinkArray& links,
            const JointVector& q, const JointVector& qDot, const JointVector& qDDot,
            const Vector3& gravity) const
    {
        auto states = ComputeForwardPass(links, q, qDot, qDDot, gravity);
        return ComputeBackwardPass(links, states);
    }

    extern template class RecursiveNewtonEuler<float, 1>;
    extern template class RecursiveNewtonEuler<float, 2>;
    extern template class RecursiveNewtonEuler<float, 3>;
}
