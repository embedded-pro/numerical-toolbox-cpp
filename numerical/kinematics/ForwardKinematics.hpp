#pragma once

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC optimize("O3", "fast-math")
#endif

#include "numerical/dynamics/RevoluteJointLink.hpp"
#include "numerical/math/CompilerOptimizations.hpp"
#include "numerical/math/Matrix.hpp"
#include <array>
#include <cmath>

namespace kinematics
{
    template<typename T, std::size_t NumLinks>
    class ForwardKinematics
    {
        static_assert(std::is_floating_point_v<T>,
            "ForwardKinematics only supports floating-point types");
        static_assert(NumLinks > 0, "Number of links must be positive");

    public:
        using Vector3 = math::Vector<T, 3>;
        using Matrix3 = math::SquareMatrix<T, 3>;
        using JointVector = math::Vector<T, NumLinks>;
        using LinkArray = std::array<dynamics::RevoluteJointLink<T>, NumLinks>;
        using PositionArray = std::array<Vector3, NumLinks + 1>;

        ForwardKinematics() = default;

        // Computes the 3D positions of all joints (including base at index 0
        // and end-effector at index NumLinks) given link descriptions and
        // joint angles.
        OPTIMIZE_FOR_SPEED PositionArray Compute(const LinkArray& links,
            const JointVector& q) const;

    private:
        static Matrix3 RotationAboutAxis(const Vector3& axis, T angle);
    };

    template<typename T, std::size_t NumLinks>
    typename ForwardKinematics<T, NumLinks>::Matrix3
    ForwardKinematics<T, NumLinks>::RotationAboutAxis(const Vector3& axis, T angle)
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
    OPTIMIZE_FOR_SPEED
        typename ForwardKinematics<T, NumLinks>::PositionArray
        ForwardKinematics<T, NumLinks>::Compute(const LinkArray& links,
            const JointVector& q) const
    {
        PositionArray positions{};

        // Base position at origin
        positions[0] = Vector3{};

        // Accumulated rotation from world to current link frame
        Matrix3 R{};
        R.at(0, 0) = T(1);
        R.at(1, 1) = T(1);
        R.at(2, 2) = T(1);

        for (std::size_t i = 0; i < NumLinks; ++i)
        {
            auto Ri = RotationAboutAxis(links[i].jointAxis, q.at(i, 0));
            R = R * Ri;

            // The link extends from its joint to the next joint.
            // parentToJoint of the next link gives the offset from this joint
            // to the next. For the last link, we use jointToCoM * 2 as an
            // approximation (end of link = joint + full link extent).
            Vector3 linkExtent;
            if (i + 1 < NumLinks)
                linkExtent = links[i + 1].parentToJoint;
            else
                linkExtent = links[i].jointToCoM * T(2);

            auto worldExtent = R * linkExtent;
            positions[i + 1] = positions[i] + worldExtent;
        }

        return positions;
    }

    extern template class ForwardKinematics<float, 1>;
    extern template class ForwardKinematics<float, 2>;
    extern template class ForwardKinematics<float, 3>;
}
