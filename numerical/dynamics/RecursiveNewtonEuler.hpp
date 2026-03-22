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
        static Vector3 CrossProduct(const Vector3& a, const Vector3& b);
        static Matrix3 RotationAboutAxis(const Vector3& axis, T angle);
    };

    template<typename T, std::size_t NumLinks>
    typename RecursiveNewtonEuler<T, NumLinks>::Vector3
    RecursiveNewtonEuler<T, NumLinks>::CrossProduct(const Vector3& a, const Vector3& b)
    {
        return Vector3{
            a.at(1, 0) * b.at(2, 0) - a.at(2, 0) * b.at(1, 0),
            a.at(2, 0) * b.at(0, 0) - a.at(0, 0) * b.at(2, 0),
            a.at(0, 0) * b.at(1, 0) - a.at(1, 0) * b.at(0, 0)
        };
    }

    // Rodrigues' rotation formula: R(axis, angle) * v
    // R = I + sin(theta)*K + (1 - cos(theta))*K^2
    // where K is the skew-symmetric matrix of 'axis'
    template<typename T, std::size_t NumLinks>
    typename RecursiveNewtonEuler<T, NumLinks>::Matrix3
    RecursiveNewtonEuler<T, NumLinks>::RotationAboutAxis(const Vector3& axis, T angle)
    {
        T c = std::cos(angle);
        T s = std::sin(angle);
        T t = T(1.0f) - c;

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
    OPTIMIZE_FOR_SPEED
        typename RecursiveNewtonEuler<T, NumLinks>::JointVector
        RecursiveNewtonEuler<T, NumLinks>::InverseDynamics(const LinkArray& links,
            const JointVector& q, const JointVector& qDot, const JointVector& qDDot,
            const Vector3& gravity) const
    {
        // Per-link spatial velocity and acceleration (in link frame)
        std::array<Vector3, NumLinks> omega;     // angular velocity
        std::array<Vector3, NumLinks> omegaDot;  // angular acceleration
        std::array<Vector3, NumLinks> linAccCoM; // linear acceleration at CoM

        // Rotation matrices from link i to parent (i-1)
        std::array<Matrix3, NumLinks> R; // R[i] rotates from link i frame to parent frame

        // ── Forward pass: propagate velocities and accelerations (base → tip) ──

        for (std::size_t i = 0; i < NumLinks; ++i)
        {
            const auto& link = links[i];

            // Rotation from link i frame to parent frame
            R[i] = RotationAboutAxis(link.jointAxis, q.at(i, 0));
            auto Rt = R[i].Transpose(); // parent-to-link rotation

            if (i == 0)
            {
                // Base link: parent is the fixed world frame
                // omega_0 = R_0^T * 0 + qDot_0 * axis_0
                omega[i] = link.jointAxis * qDot.at(i, 0);

                // omegaDot_0 = R_0^T * 0 + qDDot_0 * axis_0 + omega_0 x (qDot_0 * axis_0)
                auto qDotAxis = link.jointAxis * qDot.at(i, 0);
                omegaDot[i] = link.jointAxis * qDDot.at(i, 0) + CrossProduct(omega[i], qDotAxis);

                // Linear acceleration at joint origin (transform gravity into link frame)
                // a_0 = R_0^T * (-gravity) + omegaDot_0 x r_joint_to_origin + omega_0 x (omega_0 x r_joint_to_origin)
                // Since joint is at link origin, we propagate from the base:
                auto baseLinAcc = Rt * (gravity * T(-1.0f));

                // Acceleration at CoM
                auto rCoM = link.jointToCoM;
                linAccCoM[i] = baseLinAcc + CrossProduct(omegaDot[i], rCoM) + CrossProduct(omega[i], CrossProduct(omega[i], rCoM));
            }
            else
            {
                // Propagate from parent link i-1
                auto omegaParent = omega[i - 1];
                auto omegaDotParent = omegaDot[i - 1];

                // Transform parent angular velocity to this link frame
                auto omegaInLink = Rt * omegaParent;
                auto qDotAxis = link.jointAxis * qDot.at(i, 0);
                omega[i] = omegaInLink + qDotAxis;

                // Angular acceleration
                auto omegaDotInLink = Rt * omegaDotParent;
                omegaDot[i] = omegaDotInLink + CrossProduct(omega[i], qDotAxis) + link.jointAxis * qDDot.at(i, 0);

                // Linear acceleration at joint origin of link i
                // First get parent's joint-origin acceleration
                // Parent acceleration at its own joint origin
                auto rParentCoM = links[i - 1].jointToCoM;
                auto parentJointAcc = linAccCoM[i - 1] - CrossProduct(omegaDotParent, rParentCoM) - CrossProduct(omegaParent, CrossProduct(omegaParent, rParentCoM));

                // Acceleration at parent's joint + offset to this joint
                auto rToJoint = link.parentToJoint;
                auto accAtParentEnd = parentJointAcc + CrossProduct(omegaDotParent, rToJoint) + CrossProduct(omegaParent, CrossProduct(omegaParent, rToJoint));

                // Transform to link i frame
                auto accJointInLink = Rt * accAtParentEnd;

                // Acceleration at CoM of link i
                auto rCoM = link.jointToCoM;
                linAccCoM[i] = accJointInLink + CrossProduct(omegaDot[i], rCoM) + CrossProduct(omega[i], CrossProduct(omega[i], rCoM));
            }
        }

        // ── Backward pass: accumulate forces and torques (tip → base) ──

        std::array<Vector3, NumLinks> force;  // net force on link i (in link i frame)
        std::array<Vector3, NumLinks> torque; // net torque on link i (in link i frame)

        for (std::size_t j = NumLinks; j > 0; --j)
        {
            std::size_t i = j - 1;
            const auto& link = links[i];

            // Newton: F_i = m_i * a_CoM_i
            force[i] = linAccCoM[i] * link.mass;

            // Euler: tau_i = I_i * omegaDot_i + omega_i x (I_i * omega_i)
            auto Iw = link.inertia * omega[i];
            torque[i] = link.inertia * omegaDot[i] + CrossProduct(omega[i], Iw);

            // Shift torque to joint origin: n_i = N_i + rCoM x F_i
            auto rCoM = link.jointToCoM;
            torque[i] = torque[i] + CrossProduct(rCoM, force[i]);

            // Add contribution from child link (if any)
            if (i + 1 < NumLinks)
            {
                auto rToChild = links[i + 1].parentToJoint;

                // Rotate child force/torque from child frame to this frame
                auto childForceInParent = R[i + 1] * force[i + 1];
                auto childTorqueInParent = R[i + 1] * torque[i + 1];

                force[i] = force[i] + childForceInParent;
                torque[i] = torque[i] + childTorqueInParent + CrossProduct(rToChild, childForceInParent);
            }
        }

        // ── Extract joint torques: project torque onto joint axis ──

        JointVector tau;
        for (std::size_t i = 0; i < NumLinks; ++i)
        {
            // tau_i = axis_i^T * torque_i (dot product)
            auto axisT = links[i].jointAxis.Transpose();
            auto dotProduct = axisT * torque[i];
            tau.at(i, 0) = dotProduct.at(0, 0);
        }

        return tau;
    }

    extern template class RecursiveNewtonEuler<float, 1>;
    extern template class RecursiveNewtonEuler<float, 2>;
    extern template class RecursiveNewtonEuler<float, 3>;
}
