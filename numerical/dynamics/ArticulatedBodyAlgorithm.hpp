#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/dynamics/RevoluteJointLink.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include <array>
#include <cmath>

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

        // ── Linear algebra primitives ──

        static Vector3 CrossProduct(const Vector3& a, const Vector3& b);
        static Matrix3 RotationAboutAxis(const Vector3& axis, T angle);
        static Matrix3 SkewSymmetric(const Vector3& v);
        static Matrix3 ScaledIdentity(T value);
        static Matrix3 OuterProduct(const Vector3& a, const Vector3& b);

        // ── Pass 1: Forward kinematics ──

        static std::array<LinkKinematics, NumLinks> ComputeForwardKinematics(
            const LinkArray& links, const JointVector& q, const JointVector& qDot);

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

        // ── Pass 3: Forward accelerations ──

        static T ComputeJointAcceleration(
            const JointProjection& proj,
            const Vector3& aHatAng, const Vector3& aHatLin);
    };

    template<typename T, std::size_t NumLinks>
    typename ArticulatedBodyAlgorithm<T, NumLinks>::Vector3
    ArticulatedBodyAlgorithm<T, NumLinks>::CrossProduct(const Vector3& a, const Vector3& b)
    {
        return Vector3{
            a.at(1, 0) * b.at(2, 0) - a.at(2, 0) * b.at(1, 0),
            a.at(2, 0) * b.at(0, 0) - a.at(0, 0) * b.at(2, 0),
            a.at(0, 0) * b.at(1, 0) - a.at(1, 0) * b.at(0, 0)
        };
    }

    template<typename T, std::size_t NumLinks>
    typename ArticulatedBodyAlgorithm<T, NumLinks>::Matrix3
    ArticulatedBodyAlgorithm<T, NumLinks>::RotationAboutAxis(const Vector3& axis, T angle)
    {
        T c = std::cos(angle);
        T s = std::sin(angle);
        T t = T(1) - c;

        T x = axis.at(0, 0);
        T y = axis.at(1, 0);
        T z = axis.at(2, 0);

        return Matrix3{
            { t * x * x + c, t * x * y - s * z, t * x * z + s * y },
            { t * x * y + s * z, t * y * y + c, t * y * z - s * x },
            { t * x * z - s * y, t * y * z + s * x, t * z * z + c }
        };
    }

    template<typename T, std::size_t NumLinks>
    typename ArticulatedBodyAlgorithm<T, NumLinks>::Matrix3
    ArticulatedBodyAlgorithm<T, NumLinks>::SkewSymmetric(const Vector3& v)
    {
        return Matrix3{
            { T(0), -v.at(2, 0), v.at(1, 0) },
            { v.at(2, 0), T(0), -v.at(0, 0) },
            { -v.at(1, 0), v.at(0, 0), T(0) }
        };
    }

    template<typename T, std::size_t NumLinks>
    typename ArticulatedBodyAlgorithm<T, NumLinks>::Matrix3
    ArticulatedBodyAlgorithm<T, NumLinks>::ScaledIdentity(T value)
    {
        Matrix3 result{};
        result.at(0, 0) = value;
        result.at(1, 1) = value;
        result.at(2, 2) = value;
        return result;
    }

    template<typename T, std::size_t NumLinks>
    typename ArticulatedBodyAlgorithm<T, NumLinks>::Matrix3
    ArticulatedBodyAlgorithm<T, NumLinks>::OuterProduct(const Vector3& a, const Vector3& b)
    {
        Matrix3 result{};
        for (std::size_t r = 0; r < 3; ++r)
            for (std::size_t c = 0; c < 3; ++c)
                result.at(r, c) = a.at(r, 0) * b.at(c, 0);
        return result;
    }

    // ── Pass 1: Forward kinematics ──

    template<typename T, std::size_t NumLinks>
    std::array<typename ArticulatedBodyAlgorithm<T, NumLinks>::LinkKinematics, NumLinks>
    ArticulatedBodyAlgorithm<T, NumLinks>::ComputeForwardKinematics(
        const LinkArray& links, const JointVector& q, const JointVector& qDot)
    {
        std::array<LinkKinematics, NumLinks> kin{};

        for (std::size_t i = 0; i < NumLinks; ++i)
        {
            kin[i].R = RotationAboutAxis(links[i].jointAxis, q.at(i, 0));
            auto Rt = kin[i].R.Transpose();

            if (i == 0)
            {
                kin[i].omega = links[i].jointAxis * qDot.at(i, 0);
            }
            else
            {
                auto omegaInLink = Rt * kin[i - 1].omega;
                auto qDotAxis = links[i].jointAxis * qDot.at(i, 0);
                kin[i].omega = omegaInLink + qDotAxis;

                auto parentLinVelAtJoint = kin[i - 1].linVelJ + CrossProduct(kin[i - 1].omega, links[i].parentToJoint);
                kin[i].linVelJ = Rt * parentLinVelAtJoint;

                kin[i].cJ = CrossProduct(kin[i].omega, qDotAxis);
                kin[i].cJLin = CrossProduct(kin[i].linVelJ, qDotAxis);
            }
        }

        return kin;
    }

    // ── Pass 2 helpers ──

    template<typename T, std::size_t NumLinks>
    typename ArticulatedBodyAlgorithm<T, NumLinks>::SpatialInertia
    ArticulatedBodyAlgorithm<T, NumLinks>::ComputeRigidBodyInertia(
        const RevoluteJointLink<T>& link)
    {
        auto rSkew = SkewSymmetric(link.jointToCoM);
        auto rSkewT = rSkew.Transpose();

        return SpatialInertia{
            link.inertia + rSkewT * rSkew * link.mass,
            rSkew * link.mass,
            ScaledIdentity(link.mass)
        };
    }

    template<typename T, std::size_t NumLinks>
    typename ArticulatedBodyAlgorithm<T, NumLinks>::BiasWrench
    ArticulatedBodyAlgorithm<T, NumLinks>::ComputeBiasWrench(
        const RevoluteJointLink<T>& link,
        const LinkKinematics& kin)
    {
        auto rCoM = link.jointToCoM;
        auto velCoM = kin.linVelJ + CrossProduct(kin.omega, rCoM);
        auto biasForce = CrossProduct(kin.omega, velCoM) * link.mass;
        auto Iw = link.inertia * kin.omega;
        auto biasTorque = CrossProduct(kin.omega, Iw);

        return BiasWrench{
            biasForce,
            biasTorque + CrossProduct(rCoM, biasForce)
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
            Ia.rot - OuterProduct(proj.Ua, proj.Ua) * invD,
            Ia.cross - OuterProduct(proj.Ua, proj.UaLin) * invD,
            Ia.lin - OuterProduct(proj.UaLin, proj.UaLin) * invD
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

        auto rSkew = SkewSymmetric(parentToJoint);

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
            torqueP + CrossProduct(parentToJoint, forceP)
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
    OPTIMIZE_FOR_SPEED
        typename ArticulatedBodyAlgorithm<T, NumLinks>::JointVector
        ArticulatedBodyAlgorithm<T, NumLinks>::ForwardDynamics(const LinkArray& links,
            const JointVector& q, const JointVector& qDot, const JointVector& tau,
            const Vector3& gravity) const
    {
        // ── Pass 1: Forward kinematics (base → tip) ──

        auto kin = ComputeForwardKinematics(links, q, qDot);

        // ── Pass 2: Backward pass — articulated-body inertias (tip → base) ──

        std::array<SpatialInertia, NumLinks> Ia;
        std::array<BiasWrench, NumLinks> pA;
        std::array<JointProjection, NumLinks> proj;

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
            {
                auto IaA = ArticulatedBodyDowndate(Ia[i], proj[i]);
                auto pAA = ComputeArticulatedBiasWrench(pA[i], IaA, kin[i], proj[i]);

                auto IaParent = TransformInertiaToParent(IaA, kin[i].R, links[i].parentToJoint);
                auto pAParent = TransformWrenchToParent(pAA, kin[i].R, links[i].parentToJoint);

                Ia[i - 1].rot = Ia[i - 1].rot + IaParent.rot;
                Ia[i - 1].cross = Ia[i - 1].cross + IaParent.cross;
                Ia[i - 1].lin = Ia[i - 1].lin + IaParent.lin;
                pA[i - 1].force = pA[i - 1].force + pAParent.force;
                pA[i - 1].torque = pA[i - 1].torque + pAParent.torque;
            }
        }

        // ── Pass 3: Forward pass — joint accelerations (base → tip) ──

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
                // Spatial acceleration transform: a'_i = X_i * a_{λ(i)}
                auto rToJoint = links[i].parentToJoint;
                aPrimeAng = Rt * angAcc[i - 1];
                aPrimeLin = Rt * (linAcc[i - 1] + CrossProduct(angAcc[i - 1], rToJoint));
            }

            auto aHatAng = aPrimeAng + kin[i].cJ;
            auto aHatLin = aPrimeLin + kin[i].cJLin;

            qDDot.at(i, 0) = ComputeJointAcceleration(proj[i], aHatAng, aHatLin);

            angAcc[i] = aHatAng + links[i].jointAxis * qDDot.at(i, 0);
            linAcc[i] = aHatLin;
        }

        return qDDot;
    }

    extern template class ArticulatedBodyAlgorithm<float, 1>;
    extern template class ArticulatedBodyAlgorithm<float, 2>;
    extern template class ArticulatedBodyAlgorithm<float, 3>;
}
