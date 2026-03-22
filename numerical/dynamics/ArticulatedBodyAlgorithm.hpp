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
    // Articulated Body Algorithm (ABA): O(n) forward dynamics for serial chains.
    // Given joint positions, velocities, and applied torques, computes joint accelerations
    // directly without forming or inverting the mass matrix.
    template<typename T, std::size_t NumLinks>
    class ArticulatedBodyAlgorithm
    {
        static_assert(std::is_floating_point_v<T>,
            "ArticulatedBodyAlgorithm only supports floating-point types");
        static_assert(NumLinks > 0, "Number of links must be positive");

    public:
        using Vector3 = math::Vector<T, 3>;
        using Matrix3 = math::SquareMatrix<T, 3>;
        using JointVector = math::Vector<T, NumLinks>;
        using LinkArray = std::array<RevoluteJointLink<T>, NumLinks>;

        ArticulatedBodyAlgorithm() = default;

        OPTIMIZE_FOR_SPEED JointVector ForwardDynamics(const LinkArray& links,
            const JointVector& q, const JointVector& qDot, const JointVector& tau,
            const Vector3& gravity) const;

    private:
        struct LinkKinematics
        {
            Vector3 omega;
            Vector3 linVelJ;
            Matrix3 R;
            Vector3 cJ;
            Vector3 cJLin;
        };

        struct SpatialInertia
        {
            Matrix3 rot;
            Matrix3 cross;
            Matrix3 lin;
        };

        struct BiasWrench
        {
            Vector3 force;
            Vector3 torque;
        };

        struct JointProjection
        {
            T D;
            T u;
            Vector3 Ua;
            Vector3 UaLin;
        };

        // ── Pass 1: Forward kinematics ──

        static std::array<LinkKinematics, NumLinks> ComputeForwardKinematics(
            const LinkArray& links, const JointVector& q, const JointVector& qDot);

        static void PropagateKinematicsFromParent(
            LinkKinematics& current, const LinkKinematics& parent,
            const RevoluteJointLink<T>& link, T qDot_i);

        // ── Pass 2: Backward articulated-body inertias ──

        static SpatialInertia ComputeRigidBodyInertia(
            const RevoluteJointLink<T>& link);

        static BiasWrench ComputeBiasWrench(
            const RevoluteJointLink<T>& link,
            const LinkKinematics& kin);

        static JointProjection ComputeJointProjection(
            const Vector3& axis, const SpatialInertia& Ia,
            const BiasWrench& pA, T tau);

        static SpatialInertia ArticulatedBodyDowndate(
            const SpatialInertia& Ia, const JointProjection& proj);

        static BiasWrench ComputeArticulatedBiasWrench(
            const BiasWrench& pA, const SpatialInertia& Ia,
            const LinkKinematics& kin, const JointProjection& proj);

        static SpatialInertia TransformInertiaToParent(
            const SpatialInertia& Ia, const Matrix3& R,
            const Vector3& parentToJoint);

        static BiasWrench TransformWrenchToParent(
            const BiasWrench& wrench, const Matrix3& R,
            const Vector3& parentToJoint);

        static void AccumulateChildContribution(
            const LinkArray& links, std::size_t childIndex,
            std::array<SpatialInertia, NumLinks>& Ia,
            std::array<BiasWrench, NumLinks>& pA,
            const std::array<LinkKinematics, NumLinks>& kin,
            const std::array<JointProjection, NumLinks>& proj);

        static void ComputeArticulatedBodyInertias(
            const LinkArray& links, const JointVector& tau,
            const std::array<LinkKinematics, NumLinks>& kin,
            std::array<SpatialInertia, NumLinks>& Ia,
            std::array<BiasWrench, NumLinks>& pA,
            std::array<JointProjection, NumLinks>& proj);

        // ── Pass 3: Forward accelerations ──

        static T ComputeJointAcceleration(
            const JointProjection& proj,
            const Vector3& aHatAng, const Vector3& aHatLin);

        static JointVector ComputeJointAccelerations(
            const LinkArray& links, const Vector3& gravity,
            const std::array<LinkKinematics, NumLinks>& kin,
            const std::array<JointProjection, NumLinks>& proj);
    };

    template<typename T, std::size_t NumLinks>
    void ArticulatedBodyAlgorithm<T, NumLinks>::PropagateKinematicsFromParent(
        LinkKinematics& current, const LinkKinematics& parent,
        const RevoluteJointLink<T>& link, T qDot_i)
    {
        auto Rt = current.R.Transpose();
        auto qDotAxis = link.jointAxis * qDot_i;

        current.omega = Rt * parent.omega + qDotAxis;

        auto parentLinVelAtJoint = parent.linVelJ + math::CrossProduct(parent.omega, link.parentToJoint);
        current.linVelJ = Rt * parentLinVelAtJoint;

        current.cJ = math::CrossProduct(current.omega, qDotAxis);
        current.cJLin = math::CrossProduct(current.linVelJ, qDotAxis);
    }

    template<typename T, std::size_t NumLinks>
    std::array<typename ArticulatedBodyAlgorithm<T, NumLinks>::LinkKinematics, NumLinks>
    ArticulatedBodyAlgorithm<T, NumLinks>::ComputeForwardKinematics(
        const LinkArray& links, const JointVector& q, const JointVector& qDot)
    {
        std::array<LinkKinematics, NumLinks> kin{};

        for (std::size_t i = 0; i < NumLinks; ++i)
        {
            kin[i].R = math::RotationAboutAxis(links[i].jointAxis, q.at(i, 0));

            if (i == 0)
                kin[i].omega = links[i].jointAxis * qDot.at(i, 0);
            else
                PropagateKinematicsFromParent(kin[i], kin[i - 1], links[i], qDot.at(i, 0));
        }

        return kin;
    }

    template<typename T, std::size_t NumLinks>
    typename ArticulatedBodyAlgorithm<T, NumLinks>::SpatialInertia
    ArticulatedBodyAlgorithm<T, NumLinks>::ComputeRigidBodyInertia(
        const RevoluteJointLink<T>& link)
    {
        auto rSkew = math::SkewSymmetric(link.jointToCoM);
        auto rSkewT = rSkew.Transpose();

        return SpatialInertia{
            link.inertia + rSkewT * rSkew * link.mass,
            rSkew * link.mass,
            math::ScaledIdentity(link.mass)
        };
    }

    template<typename T, std::size_t NumLinks>
    typename ArticulatedBodyAlgorithm<T, NumLinks>::BiasWrench
    ArticulatedBodyAlgorithm<T, NumLinks>::ComputeBiasWrench(
        const RevoluteJointLink<T>& link,
        const LinkKinematics& kin)
    {
        auto rCoM = link.jointToCoM;
        auto velCoM = kin.linVelJ + math::CrossProduct(kin.omega, rCoM);
        auto biasForce = math::CrossProduct(kin.omega, velCoM) * link.mass;
        auto Iw = link.inertia * kin.omega;
        auto biasTorque = math::CrossProduct(kin.omega, Iw);

        return BiasWrench{
            biasForce,
            biasTorque + math::CrossProduct(rCoM, biasForce)
        };
    }

    template<typename T, std::size_t NumLinks>
    typename ArticulatedBodyAlgorithm<T, NumLinks>::JointProjection
    ArticulatedBodyAlgorithm<T, NumLinks>::ComputeJointProjection(
        const Vector3& axis, const SpatialInertia& Ia,
        const BiasWrench& pA, T tau)
    {
        JointProjection proj{};
        proj.Ua = Ia.rot * axis;
        proj.UaLin = Ia.cross.Transpose() * axis;

        auto dVec = axis.Transpose() * proj.Ua;
        proj.D = dVec.at(0, 0);

        auto sTpA = axis.Transpose() * pA.torque;
        proj.u = tau - sTpA.at(0, 0);

        return proj;
    }

    template<typename T, std::size_t NumLinks>
    typename ArticulatedBodyAlgorithm<T, NumLinks>::SpatialInertia
    ArticulatedBodyAlgorithm<T, NumLinks>::ArticulatedBodyDowndate(
        const SpatialInertia& Ia, const JointProjection& proj)
    {
        T invD = T(1) / proj.D;

        return SpatialInertia{
            Ia.rot - math::OuterProduct(proj.Ua, proj.Ua) * invD,
            Ia.cross - math::OuterProduct(proj.Ua, proj.UaLin) * invD,
            Ia.lin - math::OuterProduct(proj.UaLin, proj.UaLin) * invD
        };
    }

    template<typename T, std::size_t NumLinks>
    typename ArticulatedBodyAlgorithm<T, NumLinks>::BiasWrench
    ArticulatedBodyAlgorithm<T, NumLinks>::ComputeArticulatedBiasWrench(
        const BiasWrench& pA, const SpatialInertia& Ia,
        const LinkKinematics& kin, const JointProjection& proj)
    {
        auto uScaled = proj.u / proj.D;

        return BiasWrench{
            pA.force + Ia.cross.Transpose() * kin.cJ + Ia.lin * kin.cJLin + proj.UaLin * uScaled,
            pA.torque + Ia.rot * kin.cJ + Ia.cross * kin.cJLin + proj.Ua * uScaled
        };
    }

    template<typename T, std::size_t NumLinks>
    typename ArticulatedBodyAlgorithm<T, NumLinks>::SpatialInertia
    ArticulatedBodyAlgorithm<T, NumLinks>::TransformInertiaToParent(
        const SpatialInertia& Ia, const Matrix3& R,
        const Vector3& parentToJoint)
    {
        auto Rt = R.Transpose();
        auto IaRotP = R * Ia.rot * Rt;
        auto IaCrossP = R * Ia.cross * Rt;
        auto IaLinP = R * Ia.lin * Rt;

        auto rSkew = math::SkewSymmetric(parentToJoint);

        return SpatialInertia{
            IaRotP - IaCrossP * rSkew + rSkew * IaCrossP.Transpose() - rSkew * IaLinP * rSkew,
            IaCrossP + rSkew * IaLinP,
            IaLinP
        };
    }

    template<typename T, std::size_t NumLinks>
    typename ArticulatedBodyAlgorithm<T, NumLinks>::BiasWrench
    ArticulatedBodyAlgorithm<T, NumLinks>::TransformWrenchToParent(
        const BiasWrench& wrench, const Matrix3& R,
        const Vector3& parentToJoint)
    {
        auto forceP = R * wrench.force;
        auto torqueP = R * wrench.torque;

        return BiasWrench{
            forceP,
            torqueP + math::CrossProduct(parentToJoint, forceP)
        };
    }

    template<typename T, std::size_t NumLinks>
    T ArticulatedBodyAlgorithm<T, NumLinks>::ComputeJointAcceleration(
        const JointProjection& proj,
        const Vector3& aPrimeAng, const Vector3& aPrimeLin)
    {
        auto UaTa = proj.Ua.Transpose() * aPrimeAng + proj.UaLin.Transpose() * aPrimeLin;
        return (proj.u - UaTa.at(0, 0)) / proj.D;
    }

    template<typename T, std::size_t NumLinks>
    void ArticulatedBodyAlgorithm<T, NumLinks>::AccumulateChildContribution(
        const LinkArray& links, std::size_t childIndex,
        std::array<SpatialInertia, NumLinks>& Ia,
        std::array<BiasWrench, NumLinks>& pA,
        const std::array<LinkKinematics, NumLinks>& kin,
        const std::array<JointProjection, NumLinks>& proj)
    {
        auto parentIndex = childIndex - 1;

        auto IaA = ArticulatedBodyDowndate(Ia[childIndex], proj[childIndex]);
        auto pAA = ComputeArticulatedBiasWrench(pA[childIndex], IaA, kin[childIndex], proj[childIndex]);

        auto IaParent = TransformInertiaToParent(IaA, kin[childIndex].R, links[childIndex].parentToJoint);
        auto pAParent = TransformWrenchToParent(pAA, kin[childIndex].R, links[childIndex].parentToJoint);

        Ia[parentIndex].rot = Ia[parentIndex].rot + IaParent.rot;
        Ia[parentIndex].cross = Ia[parentIndex].cross + IaParent.cross;
        Ia[parentIndex].lin = Ia[parentIndex].lin + IaParent.lin;
        pA[parentIndex].force = pA[parentIndex].force + pAParent.force;
        pA[parentIndex].torque = pA[parentIndex].torque + pAParent.torque;
    }

    template<typename T, std::size_t NumLinks>
    void ArticulatedBodyAlgorithm<T, NumLinks>::ComputeArticulatedBodyInertias(
        const LinkArray& links, const JointVector& tau,
        const std::array<LinkKinematics, NumLinks>& kin,
        std::array<SpatialInertia, NumLinks>& Ia,
        std::array<BiasWrench, NumLinks>& pA,
        std::array<JointProjection, NumLinks>& proj)
    {
        for (std::size_t i = 0; i < NumLinks; ++i)
        {
            Ia[i] = ComputeRigidBodyInertia(links[i]);
            pA[i] = ComputeBiasWrench(links[i], kin[i]);
        }

        for (std::size_t j = NumLinks; j > 0; --j)
        {
            std::size_t i = j - 1;
            proj[i] = ComputeJointProjection(links[i].jointAxis, Ia[i], pA[i], tau.at(i, 0));

            if (i > 0)
                AccumulateChildContribution(links, i, Ia, pA, kin, proj);
        }
    }

    template<typename T, std::size_t NumLinks>
    typename ArticulatedBodyAlgorithm<T, NumLinks>::JointVector
    ArticulatedBodyAlgorithm<T, NumLinks>::ComputeJointAccelerations(
        const LinkArray& links, const Vector3& gravity,
        const std::array<LinkKinematics, NumLinks>& kin,
        const std::array<JointProjection, NumLinks>& proj)
    {
        JointVector qDDot;
        std::array<Vector3, NumLinks> angAcc;
        std::array<Vector3, NumLinks> linAcc;

        for (std::size_t i = 0; i < NumLinks; ++i)
        {
            auto Rt = kin[i].R.Transpose();
            Vector3 aPrimeAng;
            Vector3 aPrimeLin;

            if (i == 0)
            {
                aPrimeAng = Vector3{};
                aPrimeLin = Rt * (gravity * T(-1));
            }
            else
            {
                auto rToJoint = links[i].parentToJoint;
                aPrimeAng = Rt * angAcc[i - 1];
                aPrimeLin = Rt * (linAcc[i - 1] + math::CrossProduct(angAcc[i - 1], rToJoint));
            }

            auto aHatAng = aPrimeAng + kin[i].cJ;
            auto aHatLin = aPrimeLin + kin[i].cJLin;

            qDDot.at(i, 0) = ComputeJointAcceleration(proj[i], aHatAng, aHatLin);

            angAcc[i] = aHatAng + links[i].jointAxis * qDDot.at(i, 0);
            linAcc[i] = aHatLin;
        }

        return qDDot;
    }

    template<typename T, std::size_t NumLinks>
    OPTIMIZE_FOR_SPEED
        typename ArticulatedBodyAlgorithm<T, NumLinks>::JointVector
        ArticulatedBodyAlgorithm<T, NumLinks>::ForwardDynamics(const LinkArray& links,
            const JointVector& q, const JointVector& qDot, const JointVector& tau,
            const Vector3& gravity) const
    {
        auto kin = ComputeForwardKinematics(links, q, qDot);

        std::array<SpatialInertia, NumLinks> Ia;
        std::array<BiasWrench, NumLinks> pA;
        std::array<JointProjection, NumLinks> proj;
        ComputeArticulatedBodyInertias(links, tau, kin, Ia, pA, proj);

        return ComputeJointAccelerations(links, gravity, kin, proj);
    }

    extern template class ArticulatedBodyAlgorithm<float, 1>;
    extern template class ArticulatedBodyAlgorithm<float, 2>;
    extern template class ArticulatedBodyAlgorithm<float, 3>;
}
